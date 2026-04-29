#pragma once

#include <Arduino.h>
#include <tuple>

class PIDController {
public:
    PIDController() = default;
    PIDController(float kp, float ki, float kd) : m_kp(kp), m_ki(ki), m_kd(kd) {}

    float update(float value, float dt_s) {
        if (m_first_sample || dt_s <= 0.0f) {
            m_prev_error = m_target - value;
            m_first_sample = false;
            return m_kp * m_prev_error;
        }

        const float error = m_target - value;
        m_integral += error * dt_s;
        m_integral = constrain(m_integral, -m_integral_limit, m_integral_limit);
        const float derivative = (error - m_prev_error) / dt_s;
        m_prev_error = error;
        return m_kp * error + m_ki * m_integral + m_kd * derivative;
    }

    void reset() {
        m_integral = 0.0f;
        m_prev_error = 0.0f;
        m_first_sample = true;
    }

    void set_target(float target) { m_target = target; }
    float target() const { return m_target; }

    void set_gains(float kp, float ki, float kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    std::tuple<float, float, float> gains() const { return {m_kp, m_ki, m_kd}; }
    void set_integral_limit(float limit) { m_integral_limit = fabsf(limit); }

private:
    float m_target = 0.0f;
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_kd = 0.0f;
    float m_integral = 0.0f;
    float m_prev_error = 0.0f;
    float m_integral_limit = 1000.0f;
    bool m_first_sample = true;
};
