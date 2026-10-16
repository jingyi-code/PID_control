#ifndef HOMEWORK3_MOTOR_HPP
#define HOMEWORK3_MOTOR_HPP

#include <cstdint>
#include <PID.hpp>

class Motor
{
public:
    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);

    float angle() const;

    float speedRpm() const;
    int16_t torqueRaw() const;
    bool hasFeedback() const;

    void setTxCurrent(float amperes, uint8_t motor_id);

    uint8_t* getTxData();

private:
    const float ratio_;

    float ecd_angle_ = 0.0f;
    float speedRpm_ = 0.0f;
    int16_t torque_raw_ = 0;

    float angle_ = 0.0f;
    float last_ecd_ = 0.0f;
    bool received_ = false;

    uint8_t tx_data_[8] = {};

    static constexpr uint16_t kEncoderRange = 8192;

};

#endif
