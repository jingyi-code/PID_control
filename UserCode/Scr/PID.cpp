#include "PID.hpp"

//实现PID的控制

PID::PID(float kp, float ki, float kd,
         float i_max, float out_max, float d_filter_k)
    : kp_(kp),ki_(ki),kd_(kd)
{
    i_max_ = i_max >= 0.0f ? i_max : -i_max;
    out_max_ = out_max >= 0.0f ? out_max : -out_max;
    d_filter_k_ = limit(d_filter_k, 0.0f, 1.0f);
}

void PID::reset()
{
    error_sum_ = 0.0f;
    last_error_ = 0.0f;
    p_out_ = 0.0f;
    i_out_ = 0.0f;
    d_out_ = 0.0f;
    output_ = 0.0f;
}

void PID::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

float PID::calc(float ref, float fdb)
{
    const float error = ref - fdb;
    p_out_ = kp_ * error;
    error_sum_ = limit(error_sum_ + error, -i_max_, i_max_);
    i_out_ = ki_ * error_sum_;
    const float d_input = kd_ * (error - last_error_);

    //滤波
    d_out_ = d_out_ * (1.0f - d_filter_k_) + d_input * d_filter_k_;

    last_error_ = error;

    output_ = limit(p_out_ + i_out_ + d_out_, -out_max_, out_max_);
    return output_;
}

float PID::pOut() const { return p_out_; }
float PID::iOut() const { return i_out_; }
float PID::dOut() const { return d_out_; }
float PID::output() const { return output_; }

//限幅
float PID::limit(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}
