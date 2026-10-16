#include "can_user.h"
#include "Motor.hpp"
#include "PID.hpp"

#include "can.h"
#include "tim.h"
#include <math.h>

Motor motor(36.0f);

static constexpr uint8_t kMotorId = 1U;
static constexpr uint32_t kMotorFeedbackId = 0x200U + kMotorId;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kLoadMassKg = 0.100f;
static constexpr float kLoadArmM = 0.065f;
static constexpr float kGravity = 9.81f;
static constexpr float kTorqueConstant = 0.18f;
static constexpr float kTheoreticalFeedforwardA =
    kLoadMassKg * kGravity * kLoadArmM / kTorqueConstant;

PID position_pid(5.0f, 0.0f, 0.0f, 1000.0f, 150.0f, 0.05f);
PID speed_pid(0.3f, 0.0f, 0.0f, 5000.0f, 10.0f, 0.05f);

volatile uint8_t stop_flag = 1U;

volatile uint8_t control_mode = 1U;

volatile float speed_test_target_rpm = 0.0f;
volatile float target_angle = 0.0f;

volatile float position_kp = 5.0f;
volatile float position_ki = 0.0f;

volatile float position_kd = 0.7f;

volatile float position_d_filter_k = 0.15f;

volatile float speed_kp = 0.3f;
volatile float speed_ki = 0.0f;
volatile float speed_kd = 0.0f;

volatile float feedforward_scale = 1.6f;
volatile float feedforward_direction = 1.0f;

volatile float current_limit_a = 8.0f;

volatile float feedback_angle_debug = 0.0f;
volatile float feedback_speed_debug = 0.0f;
volatile float angle_error_debug = 0.0f;
volatile float target_speed_debug = 0.0f;
volatile float position_p_out_debug = 0.0f;
volatile float position_i_out_debug = 0.0f;
volatile float position_d_out_debug = 0.0f;
volatile float speed_p_out_debug = 0.0f;
volatile float speed_i_out_debug = 0.0f;
volatile float speed_d_out_debug = 0.0f;
volatile float pid_current_debug = 0.0f;
volatile float feedforward_current_debug = 0.0f;
volatile float current_debug = 0.0f;

volatile uint32_t test_elapsed_ms = 0U;
volatile uint32_t settling_time_ms = 0U;
volatile uint16_t tolerance_count_ms = 0U;
volatile uint8_t within_tolerance = 0U;
static float last_target_angle = 0.0f;
static uint8_t last_control_mode = 1U;
static float position_damping_state = 0.0f;

static float limitFloat(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float absFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

static float calcGravityFeedforward(float angle_deg)
{
    const float angle_rad = angle_deg * kPi / 180.0f;
    return feedforward_direction
           * feedforward_scale
           * kTheoreticalFeedforwardA
           * sinf(angle_rad);
}

static void clearDebugOutputs()
{
    target_speed_debug = 0.0f;
    position_p_out_debug = 0.0f;
    position_i_out_debug = 0.0f;
    position_d_out_debug = 0.0f;
    speed_p_out_debug = 0.0f;
    speed_i_out_debug = 0.0f;
    speed_d_out_debug = 0.0f;
    pid_current_debug = 0.0f;
    feedforward_current_debug = 0.0f;
    current_debug = 0.0f;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    uint8_t data[8];
    if (hcan != &hcan1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
        return;
    }

    if (rx_header.IDE == CAN_ID_STD
        && rx_header.RTR == CAN_RTR_DATA
        && rx_header.DLC == 8U
        && rx_header.StdId == kMotorFeedbackId)
    {
        motor.canRxMsgCallback(data); //将收到的数组交给Motor类解析
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance != TIM6) return;

    feedback_angle_debug = motor.angle();
    feedback_speed_debug = motor.speedRpm();
    angle_error_debug = target_angle - feedback_angle_debug;

    if (stop_flag == 1U || !motor.hasFeedback())
    {
        position_pid.reset();
        speed_pid.reset();
        position_damping_state = 0.0f;
        clearDebugOutputs();
        test_elapsed_ms = 0U;
        settling_time_ms = 0U;
        tolerance_count_ms = 0U;
        within_tolerance = 0U;
        last_target_angle = target_angle;
        last_control_mode = control_mode;
        motor.setTxCurrent(0.0f, kMotorId);
    }
    else
    {

        position_pid.setGains(position_kp, position_ki, 0.0f);
        speed_pid.setGains(speed_kp, speed_ki, speed_kd);

        if (control_mode != last_control_mode)  //开始位置环的调参
        {
            position_pid.reset();
            speed_pid.reset();
            position_damping_state = 0.0f;
            test_elapsed_ms = 0U;
            settling_time_ms = 0U;
            tolerance_count_ms = 0U;
            within_tolerance = 0U;
            last_target_angle = target_angle;
            last_control_mode = control_mode;
        }

        if (control_mode == 1U) //位置环相关参数置零
        {

            position_pid.reset();
            position_damping_state = 0.0f;
            target_speed_debug = speed_test_target_rpm;
            position_p_out_debug = 0.0f;
            position_i_out_debug = 0.0f;
            position_d_out_debug = 0.0f;
            feedforward_current_debug = 0.0f;

            test_elapsed_ms = 0U;
            settling_time_ms = 0U;
            tolerance_count_ms = 0U;
            within_tolerance = 0U;
            last_target_angle = target_angle;
        }
        else
        {

            if (target_angle != last_target_angle)
            {
                position_pid.reset();
                speed_pid.reset();
                position_damping_state = 0.0f;
                test_elapsed_ms = 0U;
                settling_time_ms = 0U;
                tolerance_count_ms = 0U;
                within_tolerance = 0U;
                last_target_angle = target_angle;
            }

            const float position_pi_output = position_pid.calc(
                target_angle, feedback_angle_debug);

            const float d_filter = limitFloat(
                position_d_filter_k, 0.0f, 1.0f);
            const float damping_input =
                -position_kd * feedback_speed_debug;
            position_damping_state =
                position_damping_state * (1.0f - d_filter)
                + damping_input * d_filter;

            target_speed_debug = limitFloat(
                position_pi_output + position_damping_state,
                -150.0f,
                150.0f);
            position_p_out_debug = position_pid.pOut();
            position_i_out_debug = position_pid.iOut();
            position_d_out_debug = position_damping_state;
            feedforward_current_debug =
                calcGravityFeedforward(feedback_angle_debug);
        }

        pid_current_debug = speed_pid.calc(
            target_speed_debug, feedback_speed_debug);
        speed_p_out_debug = speed_pid.pOut();
        speed_i_out_debug = speed_pid.iOut();
        speed_d_out_debug = speed_pid.dOut();

        const float safe_current_limit = limitFloat(
            current_limit_a, 0.0f, 8.0f);
        current_debug = limitFloat(
            pid_current_debug + feedforward_current_debug,
            -safe_current_limit,
            safe_current_limit);
        motor.setTxCurrent(current_debug, kMotorId);

        if (control_mode == 0U)
        {
            ++test_elapsed_ms;
            if (absFloat(angle_error_debug) <= 0.1f)
            {
                if (tolerance_count_ms < 20U) ++tolerance_count_ms;
                if (tolerance_count_ms >= 20U)
                {
                    within_tolerance = 1U;
                    if (settling_time_ms == 0U)
                    {
                        settling_time_ms = test_elapsed_ms - 19U;
                    }
                }
            }
            else
            {
                tolerance_count_ms = 0U;
                within_tolerance = 0U;
            }
        }
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        HAL_CAN_AddTxMessage(
            &hcan1, &tx_header, motor.getTxData(), &can_tx_mailbox);
    }
}
