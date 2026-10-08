#include "ControlTask.h"

#include "Gm6020.h"
#include "Pid.h"

#include "can.h"
#include "iwdg.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#include <cmath>
#include <cstring>

namespace {

constexpr uint8_t  kMotorId    = 1;
constexpr float    kVoltLimit  = 25000.0f;
constexpr float    kMaxTemp    = 75.0f;
constexpr uint32_t kFdbTimeout = 100;

enum class TestMode : uint8_t {
    PositionStep = 2,
    VelocitySine = 4
};

constexpr float kVelMax = 15.0f;
constexpr float kPosKp  = 13.0f;
constexpr float kPosKi  = 0.5f;
constexpr float kPosKd  = 0.0f;

constexpr float kVelKp = 600.0f;
constexpr float kVelKi = 200.0f;
constexpr float kVelKd = 0.3f;

constexpr float kVelLpfA = 0.30f;
constexpr float kVelFf   = 750.0f;

constexpr uint32_t kPlotDiv     = 10;
constexpr uint32_t kTxTimeout   = 20;
constexpr uint32_t kPlotChans   = 8;
constexpr uint32_t kPlotPayload = kPlotChans * 4;
constexpr uint32_t kPlotLen     = kPlotPayload + 4;

Gm6020 g_motor(kMotorId);
Pid    g_pid_pos(kPosKp, kPosKi, kPosKd, kVelMax, -kVelMax);
Pid    g_pid_vel(kVelKp, kVelKi, kVelKd, kVoltLimit, -kVoltLimit);

volatile uint32_t g_tick         = 0;
volatile uint32_t g_last_rx_tick = 0;
volatile uint8_t  g_rx_flag      = 0;
volatile uint8_t  g_target_ready = 0;
volatile float    g_target_angle = 0.0f;
volatile float    g_zero_angle   = 0.0f;
volatile float    g_vel_ref      = 0.0f;
volatile float    g_pid_out      = 0.0f;
volatile float    g_vel_filt     = 0.0f;

uint8_t           g_plot_buf[kPlotLen];
volatile uint8_t  g_tx_busy = 0;
volatile uint32_t g_tx_tick = 0;

void sendVoltageFrame() {
    CAN_TxHeaderTypeDef tx {};
    tx.StdId = g_motor.txId();
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;
    tx.TransmitGlobalTime = DISABLE;

    uint8_t data[8] = {0};
    g_motor.encode(data);

    uint32_t mailbox = 0;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        (void)HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
    }
}

}

volatile uint8_t g_test_mode        = 4;
volatile float   g_pos_target_deg   = 90.0f;
volatile float   g_sine_freq_hz     = 0.5f;
volatile float   g_vel_sine_amp_rpm = 50.0f;

volatile float g_pos_kp    = kPosKp;
volatile float g_pos_ki    = kPosKi;
volatile float g_pos_kd    = kPosKd;
volatile float g_vel_kp    = kVelKp;
volatile float g_vel_ki    = kVelKi;
volatile float g_vel_kd    = kVelKd;
volatile float g_vel_lpf_a = kVelLpfA;
volatile float g_vel_ff    = kVelFf;

volatile uint32_t dbg_tick         = 0;
volatile float    dbg_target_pos   = 0.0f;
volatile float    dbg_fdb_pos      = 0.0f;
volatile float    dbg_vel_ref_rpm  = 0.0f;
volatile float    dbg_fdb_vel_rpm  = 0.0f;
volatile float    dbg_vel_filt_rpm = 0.0f;
volatile float    dbg_fdb_vel_rads = 0.0f;
volatile float    dbg_u_voltage    = 0.0f;
volatile float    dbg_u_ff         = 0.0f;
volatile float    dbg_u_pid        = 0.0f;
volatile float    dbg_current_a    = 0.0f;
volatile float    dbg_temp_c       = 0.0f;
volatile int16_t  dbg_raw_vel_rpm  = 0;
volatile uint32_t dbg_rx_age_ms    = 0;

void ControlTaskInit(void) {
    CAN_FilterTypeDef filter {};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) { Error_Handler(); }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)                 { Error_Handler(); }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    if (hcan2.Instance != nullptr) {
        filter.FilterBank           = 14;
        filter.FilterFIFOAssignment = CAN_RX_FIFO1;
        if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) { Error_Handler(); }
        if (HAL_CAN_Start(&hcan2) != HAL_OK)                 { Error_Handler(); }
        if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
            Error_Handler();
        }
    }

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) { Error_Handler(); }
}

void MainTask(void) {
    ++g_tick;

    const TestMode mode = static_cast<TestMode>(g_test_mode);

    if (!g_target_ready && g_rx_flag) {
        g_zero_angle   = g_motor.angle();
        g_target_ready = 1;
        g_pid_pos.reset();
        g_pid_vel.reset();
    }

    if (mode == TestMode::VelocitySine) {
        g_vel_ref = g_vel_sine_amp_rpm * 0.1047198f *
                    sinf(6.2831853f * g_sine_freq_hz * static_cast<float>(g_tick) * 0.001f);
    } else {
        g_target_angle = g_zero_angle + g_pos_target_deg * 0.0174533f;
    }

    g_vel_filt += g_vel_lpf_a * (g_motor.vel() - g_vel_filt);

    g_pid_pos.setParams(g_pos_kp, g_pos_ki, g_pos_kd);
    g_pid_vel.setParams(g_vel_kp, g_vel_ki, g_vel_kd);

    const bool fdb_alive = g_rx_flag && ((g_tick - g_last_rx_tick) < kFdbTimeout);
    const bool over_temp = (g_motor.temp() > kMaxTemp);

    float u    = 0.0f;
    float u_ff = 0.0f;

    if (!g_target_ready || !fdb_alive || over_temp) {
        g_pid_pos.reset();
        g_pid_vel.reset();
        g_vel_ref  = 0.0f;
        g_vel_filt = g_motor.vel();
    } else if (mode == TestMode::PositionStep) {
        g_vel_ref = g_pid_pos.calc(g_target_angle, g_motor.angle());
        u_ff      = g_vel_ff * g_vel_ref;
        u         = u_ff + g_pid_vel.calc(g_vel_ref, g_vel_filt);
    } else {
        u_ff = g_vel_ff * g_vel_ref;
        u    = u_ff + g_pid_vel.calc(g_vel_ref, g_vel_filt);
    }

    if (u >  kVoltLimit) { u =  kVoltLimit; }
    if (u < -kVoltLimit) { u = -kVoltLimit; }

    g_pid_out = u;
    g_motor.setVoltage(static_cast<int16_t>(u));
    sendVoltageFrame();

    dbg_tick         = g_tick;
    dbg_target_pos   = g_target_angle * 57.29578f;
    dbg_fdb_pos      = g_motor.angle() * 57.29578f;
    dbg_vel_ref_rpm  = g_vel_ref * 9.549297f;
    dbg_fdb_vel_rpm  = g_motor.vel() * 9.549297f;
    dbg_vel_filt_rpm = g_vel_filt * 9.549297f;
    dbg_fdb_vel_rads = g_motor.vel();
    dbg_u_voltage    = u;
    dbg_u_ff         = u_ff;
    dbg_u_pid        = u - u_ff;
    dbg_current_a    = g_motor.current();
    dbg_temp_c       = g_motor.temp();
    dbg_raw_vel_rpm  = g_motor.velRpm();
    dbg_rx_age_ms    = g_tick - g_last_rx_tick;

    HAL_IWDG_Refresh(&hiwdg);
}

void PlotTask(void) {
    if ((g_tick % kPlotDiv) != 0) { return; }
    if (g_tx_busy && ((g_tick - g_tx_tick) < kTxTimeout)) { return; }
    if (huart1.gState != HAL_UART_STATE_READY) { return; }

    constexpr float kRadToDeg = 57.29578f;
    constexpr float kRadToRpm = 9.549297f;

    float f[kPlotChans];
    f[0] = static_cast<float>(g_tick);
    f[1] = g_target_angle * kRadToDeg;
    f[2] = g_motor.angle() * kRadToDeg;
    f[3] = g_vel_ref * kRadToRpm;
    f[4] = g_motor.vel() * kRadToRpm;
    f[5] = g_pid_out;
    f[6] = g_motor.current();
    f[7] = static_cast<float>(g_tick - g_last_rx_tick);

    memcpy(g_plot_buf, f, kPlotPayload);
    g_plot_buf[kPlotPayload + 0] = 0x00;
    g_plot_buf[kPlotPayload + 1] = 0x00;
    g_plot_buf[kPlotPayload + 2] = 0x80;
    g_plot_buf[kPlotPayload + 3] = 0x7F;

    g_tx_tick = g_tick;
    if (HAL_UART_Transmit_DMA(&huart1, g_plot_buf, kPlotLen) == HAL_OK) {
        g_tx_busy = 1;
    }
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id != g_motor.rxId()) { return; }
    g_motor.decode(data);
    g_last_rx_tick = g_tick;
    g_rx_flag      = 1;
}

extern "C" {

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        g_tx_busy = 0;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx {};
    uint8_t data[8] = {0};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) { break; }
        if (rx.IDE == CAN_ID_STD) {
            CanFeedbackCallback(rx.StdId, data);
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx {};
    uint8_t data[8] = {0};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx, data) != HAL_OK) { break; }
        if (rx.IDE == CAN_ID_STD) {
            CanFeedbackCallback(rx.StdId, data);
        }
    }
}

}
