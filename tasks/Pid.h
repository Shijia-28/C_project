#ifndef PID_H
#define PID_H

class Pid {
public:
    Pid(float kp, float ki, float kd, float out_max, float out_min);
    ~Pid() = default;

    void setParams(float kp, float ki, float kd);
    void setOutputLimit(float out_max, float out_min);
    void reset();

    float calc(float ref, float fdb);

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_, ki_, kd_;
    float out_max_, out_min_;
    float integ_;
    float last_err_;
};

#endif
