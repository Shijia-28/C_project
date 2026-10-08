#include "Pid.h"

namespace {
constexpr float kSampleTime = 0.001f;
}

Pid::Pid(float kp, float ki, float kd, float out_max, float out_min)
    : kp_(kp), ki_(ki), kd_(kd),
      out_max_(out_max), out_min_(out_min),
      integ_(0.0f), last_err_(0.0f) {}

void Pid::setParams(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void Pid::setOutputLimit(float out_max, float out_min) {
    out_max_ = out_max; out_min_ = out_min;
}

void Pid::reset() {
    integ_ = 0.0f;
    last_err_ = 0.0f;
}

float Pid::calc(float ref, float fdb) {
    float err   = ref - fdb;
    float deriv = err - last_err_;

    integ_ += err;

    float out = kp_ * err
              + ki_ * integ_ * kSampleTime
              + kd_ * deriv / kSampleTime;

    if (out > out_max_) {
        out = out_max_;
        integ_ -= err;
    } else if (out < out_min_) {
        out = out_min_;
        integ_ -= err;
    }

    last_err_ = err;
    return out;
}
