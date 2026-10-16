#ifndef HOMEWORK3_PID_HPP
#define HOMEWORK3_PID_HPP

class PID
{
public:
    PID(float kp, float ki, float kd,
        float i_max, float out_max, float d_filter_k);

    void reset();
    void setGains(float kp, float ki, float kd);

    float calc(float ref, float fdb);

    float pOut() const;
    float iOut() const;
    float dOut() const;
    float output() const;

private:
    static float limit(float value, float min_value, float max_value);

    float kp_;
    float ki_;
    float kd_;
    float i_max_;
    float out_max_ = 0.0f;
    float d_filter_k_ = 0.0f;
    float error_sum_ = 0.0f;
    float last_error_ = 0.0f;
    float p_out_ = 0.0f;
    float i_out_ = 0.0f;
    float d_out_ = 0.0f;
    float output_ = 0.0f;
};

#endif
