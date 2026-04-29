#include <Arduino.h>

#include "encoder_motor.hpp"
#include "robot_config.hpp"
#include "serial_logger.hpp"
#include "task_scheduler.hpp"
#include "wifi_udp_comm.hpp"

namespace {

TaskScheduler scheduler;
WifiUdpComms comms(robot_config::UDP_PORT);

EncoderMotor left_motor(
    robot_config::LEFT_ENC_A,
    robot_config::LEFT_ENC_B,
    robot_config::LEFT_PWM,
    robot_config::LEFT_IN_A,
    robot_config::LEFT_IN_B,
    robot_config::LEFT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

EncoderMotor right_motor(
    robot_config::RIGHT_ENC_A,
    robot_config::RIGHT_ENC_B,
    robot_config::RIGHT_PWM,
    robot_config::RIGHT_IN_A,
    robot_config::RIGHT_IN_B,
    robot_config::RIGHT_ENABLE,
    robot_config::MOTOR_GEAR_RATIO,
    robot_config::MOTOR_RAW_CPR);

struct DrivePacket {
    float left_rad_s;
    float right_rad_s;
};

void update_motors() {
    constexpr float dt_s = 0.02f;
    left_motor.update_velocity(dt_s);
    right_motor.update_velocity(dt_s);
    left_motor.tick_control(dt_s);
    right_motor.tick_control(dt_s);
}

void poll_udp() {
    uint8_t buffer[64];
    const int len = comms.read(buffer, sizeof(buffer));
    if (len == static_cast<int>(sizeof(DrivePacket))) {
        const DrivePacket* packet = reinterpret_cast<const DrivePacket*>(buffer);
        left_motor.set_target_rad_s(packet->left_rad_s);
        right_motor.set_target_rad_s(packet->right_rad_s);
        const char ack[] = "OK drive";
        comms.send_reply(reinterpret_cast<const uint8_t*>(ack), sizeof(ack) - 1);
    }
}

void print_status() {
    Serial.print("left count=");
    Serial.print(left_motor.count());
    Serial.print(" vel=");
    Serial.print(left_motor.velocity_rad_s());
    Serial.print(" | right count=");
    Serial.print(right_motor.count());
    Serial.print(" vel=");
    Serial.println(right_motor.velocity_rad_s());
}

}  // namespace

void robot_app_setup() {
    log_begin();
    LOG_INFO("term3_robot starting");

    left_motor.begin();
    right_motor.begin();
    left_motor.set_pid(30.0f, 5.0f, 0.0f);
    right_motor.set_pid(30.0f, 5.0f, 0.0f);

    comms.begin(robot_config::WIFI_SSID, robot_config::WIFI_PASSWORD);

    scheduler.add(20, update_motors, "motors");
    scheduler.add(50, poll_udp, "udp");
    scheduler.add(500, print_status, "status");
    scheduler.reset();
}

void robot_app_loop() {
    scheduler.tick(millis());
}
