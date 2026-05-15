#include "encoder_motor.hpp"

EncoderMotor* EncoderMotor::s_left_instance = nullptr;
EncoderMotor* EncoderMotor::s_right_instance = nullptr;

EncoderMotor::EncoderMotor(
    uint8_t enc_a,
    uint8_t enc_b,
    uint8_t pwm_pin,
    uint8_t in_a_pin,
    uint8_t in_b_pin,
    uint8_t enable_pin,
    float gear_ratio,
    float raw_cpr)
    : m_enc_a(enc_a),
      m_enc_b(enc_b),
      m_pwm_pin(pwm_pin),
      m_in_a_pin(in_a_pin),
      m_in_b_pin(in_b_pin),
      m_enable_pin(enable_pin),
      m_gear_ratio(gear_ratio),
      m_raw_cpr(raw_cpr) {}

void EncoderMotor::begin() {
    begin_encoder_only();
    pinMode(m_pwm_pin, OUTPUT);
    pinMode(m_in_a_pin, OUTPUT);
    pinMode(m_in_b_pin, OUTPUT);
    pinMode(m_enable_pin, INPUT_PULLUP);
    stop();
}

void EncoderMotor::begin_encoder_only() {
    begin_encoder_inputs();

    if (s_left_instance == this || s_right_instance == this) {
        return;
    }

    const int interrupt_number = digitalPinToInterrupt(m_enc_a);
    if (interrupt_number == NOT_AN_INTERRUPT) {
        return;
    }

    if (s_left_instance == nullptr) {
        s_left_instance = this;
        attachInterrupt(interrupt_number, EncoderMotor::isr_left, CHANGE);
    } else if (s_right_instance == nullptr) {
        s_right_instance = this;
        attachInterrupt(interrupt_number, EncoderMotor::isr_right, CHANGE);
    }
}

void EncoderMotor::begin_encoder_polling_only() {
    begin_encoder_inputs();
}

void EncoderMotor::poll_encoder() {
    if (!m_encoder_inputs_started) {
        begin_encoder_polling_only();
    }

    const bool enc_a = digitalRead(m_enc_a);
    if (enc_a == m_prev_enc_a) {
        return;
    }

    m_prev_enc_a = enc_a;
    handle_interrupt();
}

void EncoderMotor::update_velocity(float dt_s) {
    noInterrupts();
    const long count = m_count;
    interrupts();

    const long delta = count - m_prev_count;
    m_prev_count = count;

    if (dt_s <= 0.0f) {
        return;
    }

    const float rad_per_count = TWO_PI / (m_gear_ratio * m_raw_cpr);
    const float raw_velocity = delta * rad_per_count / dt_s;
    constexpr float alpha = 0.3f;
    m_velocity_rad_s = alpha * raw_velocity + (1.0f - alpha) * m_velocity_rad_s;
}

void EncoderMotor::set_target_rad_s(float target_rad_s) {
    m_pid.set_target(target_rad_s);
}

void EncoderMotor::set_pid(float kp, float ki, float kd) {
    m_pid.set_gains(kp, ki, kd);
}

void EncoderMotor::set_pwm_limit(int limit) {
    m_pwm_limit = constrain(limit, 0, 255);
}

void EncoderMotor::stop() {
    analogWrite(m_pwm_pin, 0);
    digitalWrite(m_in_a_pin, LOW);
    digitalWrite(m_in_b_pin, LOW);
}

void EncoderMotor::tick_control(float dt_s) {
    const int power = static_cast<int>(m_pid.update(m_velocity_rad_s, dt_s));
    write_power(constrain(power, -m_pwm_limit, m_pwm_limit));
}

void EncoderMotor::isr_left() {
    if (s_left_instance != nullptr) {
        s_left_instance->handle_interrupt();
    }
}

void EncoderMotor::isr_right() {
    if (s_right_instance != nullptr) {
        s_right_instance->handle_interrupt();
    }
}

void EncoderMotor::begin_encoder_inputs() {
    pinMode(m_enc_a, INPUT_PULLUP);
    pinMode(m_enc_b, INPUT_PULLUP);
    m_prev_enc_a = digitalRead(m_enc_a);
    m_encoder_inputs_started = true;
}

void EncoderMotor::handle_interrupt() {
    if (digitalRead(m_enc_a) != digitalRead(m_enc_b)) {
        ++m_count;
    } else {
        --m_count;
    }
}

void EncoderMotor::write_power(int power) {
    if (power == 0) {
        stop();
        return;
    }

    digitalWrite(m_in_a_pin, power > 0 ? HIGH : LOW);
    digitalWrite(m_in_b_pin, power > 0 ? LOW : HIGH);
    analogWrite(m_pwm_pin, abs(power));
}
