#include "Gm6020.h"

namespace {
constexpr float    kPi         = 3.14159265358979323846f;
constexpr uint16_t kAngleRange = 8192;
constexpr float    kRpmToRadS  = (2.0f * kPi) / 60.0f;
constexpr float    kRawToAmp   = 3.0f / 16384.0f;
}

Gm6020::Gm6020(uint8_t id)
    : id_(id), voltage_(0),
      raw_angle_(0), raw_vel_rpm_(0), raw_current_(0), raw_temp_(0),
      full_turns_(0), last_raw_angle_(0) {}

uint32_t Gm6020::rxId() const {
    return 0x204u + id_;
}

uint32_t Gm6020::txId() const {
    return (id_ <= 4) ? 0x1FFu : 0x2FFu;
}

void Gm6020::decode(const uint8_t *data) {
    uint16_t raw = static_cast<uint16_t>((data[0] << 8) | data[1]);

    int16_t diff = static_cast<int16_t>(raw - last_raw_angle_);
    if (diff > 4096)        full_turns_--;
    else if (diff < -4096)  full_turns_++;

    last_raw_angle_ = raw;
    raw_angle_      = raw;
    raw_vel_rpm_    = static_cast<int16_t>((data[2] << 8) | data[3]);
    raw_current_    = static_cast<int16_t>((data[4] << 8) | data[5]);
    raw_temp_       = data[6];
}

float Gm6020::angle() const {

    return (static_cast<float>(full_turns_) * kAngleRange + raw_angle_)
           * (2.0f * kPi / kAngleRange);
}

float Gm6020::vel() const {
    return raw_vel_rpm_ * kRpmToRadS;
}

float Gm6020::current() const {
    return raw_current_ * kRawToAmp;
}

float Gm6020::temp() const {
    return static_cast<float>(raw_temp_);
}

int16_t Gm6020::velRpm() const {
    return raw_vel_rpm_;
}

void Gm6020::setVoltage(int16_t v) {
    voltage_ = v;
}

int16_t Gm6020::voltage() const {
    return voltage_;
}

void Gm6020::encode(uint8_t *data) const {

    uint8_t index = (id_ <= 4) ? static_cast<uint8_t>((id_ - 1) * 2)
                               : static_cast<uint8_t>((id_ - 5) * 2);
    data[index]     = static_cast<uint8_t>((voltage_ >> 8) & 0xFF);
    data[index + 1] = static_cast<uint8_t>(voltage_ & 0xFF);
}
