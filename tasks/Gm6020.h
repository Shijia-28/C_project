#ifndef GM6020_H
#define GM6020_H

#include <cstdint>

class Gm6020 {
public:

    explicit Gm6020(uint8_t id);
    ~Gm6020() = default;

    uint32_t rxId() const;
    uint32_t txId() const;

    float angle() const;
    float vel() const;
    float current() const;
    float temp() const;
    int16_t velRpm() const;

    void setVoltage(int16_t v);
    int16_t voltage() const;

    void decode(const uint8_t *data);
    void encode(uint8_t *data) const;

private:
    uint8_t  id_;
    int16_t  voltage_;

    uint16_t raw_angle_;
    int16_t  raw_vel_rpm_;
    int16_t  raw_current_;
    uint8_t  raw_temp_;

    int32_t  full_turns_;
    uint16_t last_raw_angle_;
};

#endif
