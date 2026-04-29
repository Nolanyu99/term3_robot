#pragma once

#include <Arduino.h>
#include "pid_controller.hpp"

class EncoderMotor {
public:
    EncoderMotor(
        uint8_t enc_a,
        uint8_t enc_b,
        uint8_t pwm_pin,
        uint8_t in_a_pin,
        uint8_t in_b_pin,
        uint8_t enable_pin,
        float gear_ratio,
        float raw_cpr);

    void begin();
    void update_velocity(float dt_s);
    void set_target_rad_s(float target_rad_s);
    void set_pid(float kp, float ki, float kd);
    void set_pwm_limit(int limit);
    void stop();
    void tick_control(float dt_s);

    float velocity_rad_s() const { return m_velocity_rad_s; }
    long count() const { return m_count; }

private:
    static void isr_left();
    static void isr_right();
    void handle_interrupt();
    void write_power(int power);

    static EncoderMotor* s_left_instance;
    static EncoderMotor* s_right_instance;

    uint8_t m_enc_a;
    uint8_t m_enc_b;
    uint8_t m_pwm_pin;
    uint8_t m_in_a_pin;
    uint8_t m_in_b_pin;
    uint8_t m_enable_pin;
    float m_gear_ratio;
    float m_raw_cpr;
    PIDController m_pid;
    volatile long m_count = 0;
    long m_prev_count = 0;
    float m_velocity_rad_s = 0.0f;
    int m_pwm_limit = 180;
};
