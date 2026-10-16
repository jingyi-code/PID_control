#include "Motor.hpp"

Motor::Motor(float ratio)
    : ratio_(ratio)
{
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    const uint16_t ecd = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    const int16_t speed_raw = static_cast<int16_t>((static_cast<uint16_t>(rx_data[2]) << 8) | rx_data[3]);
    const int16_t torque_raw = static_cast<int16_t>((static_cast<uint16_t>(rx_data[4]) << 8) | rx_data[5]);

    ecd_angle_ = static_cast<float>(ecd) * 360.0f / kEncoderRange;

    speedRpm_ = static_cast<float>(speed_raw) / ratio_;
    torque_raw_ = torque_raw;

    if (!received_)
    {
        last_ecd_ = static_cast<float>(ecd);
        received_ = true;
        return;
    }

    float delta = static_cast<float>(ecd) - last_ecd_;

    if (delta > kEncoderRange / 2.0f)
    {
        delta -= kEncoderRange;
    }
    else if (delta < -kEncoderRange / 2.0f)
    {
        delta += kEncoderRange;
    }

    angle_ += delta * 360.0f / kEncoderRange / ratio_;
    last_ecd_ = static_cast<float>(ecd);
}

float Motor::angle() const
{
    return angle_;
}

float Motor::speedRpm() const
{
    return speedRpm_;
}

int16_t Motor::torqueRaw() const
{
    return torque_raw_;
}

bool Motor::hasFeedback() const
{
    return received_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id)
{

    if (motor_id < 1U || motor_id > 4U)
    {
        return;
    }

    if (amperes > 10.0f)
    {
        amperes = 10.0f;
    }
    else if (amperes < -10.0f)
    {
        amperes = -10.0f;
    }

    const int16_t current_command = static_cast<int16_t>(amperes * 1000.0f);
    const uint16_t packed = static_cast<uint16_t>(current_command);

    //写发送的数组
    const uint8_t index = static_cast<uint8_t>((motor_id - 1U) * 2U);
    tx_data_[index] = static_cast<uint8_t>(packed >> 8);
    tx_data_[index + 1U] = static_cast<uint8_t>(packed & 0xFFU);
}

uint8_t* Motor::getTxData()
{
    return tx_data_;
}
