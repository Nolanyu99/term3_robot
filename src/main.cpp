#include <Arduino.h>

#ifndef APP_MODE
#define APP_MODE 2
#endif

void robot_app_setup();
void robot_app_loop();
void wifi_test_app_setup();
void wifi_test_app_loop();
void motor_test_app_setup();
void motor_test_app_loop();
void upload_test_app_setup();
void upload_test_app_loop();
void encoder_test_app_setup();
void encoder_test_app_loop();
void qtr_test_app_setup();
void qtr_test_app_loop();
void button_led_test_app_setup();
void button_led_test_app_loop();
void simple_motoron_test_app_setup();
void simple_motoron_test_app_loop();
void ultrasonic_test_app_setup();
void ultrasonic_test_app_loop();
void servo_test_app_setup();
void servo_test_app_loop();
void rfid_test_app_setup();
void rfid_test_app_loop();
void ir_test_app_setup();
void ir_test_app_loop();
void qtr_lib_test_app_setup();
void qtr_lib_test_app_loop();
void mini_messenger_test_app_setup();
void mini_messenger_test_app_loop();
void motor_messenger_test_app_setup();
void motor_messenger_test_app_loop();
void motor_messenger_button_led_app_setup();
void motor_messenger_button_led_app_loop();
void ir_test_updated_app_setup();
void ir_test_updated_app_loop();
void line_follow_test_app_setup();
void line_follow_test_app_loop();

void setup() {
#if APP_MODE == 0
    robot_app_setup();
#elif APP_MODE == 1
    wifi_test_app_setup();
#elif APP_MODE == 2
    motor_test_app_setup();
#elif APP_MODE == 3
    upload_test_app_setup();
#elif APP_MODE == 4
    encoder_test_app_setup();
#elif APP_MODE == 5
    qtr_test_app_setup();
#elif APP_MODE == 6
    button_led_test_app_setup();
#elif APP_MODE == 7
    simple_motoron_test_app_setup();
#elif APP_MODE == 8
    ultrasonic_test_app_setup();
#elif APP_MODE == 9
    servo_test_app_setup();
#elif APP_MODE == 10
    rfid_test_app_setup();
#elif APP_MODE == 11
    ir_test_app_setup();
#elif APP_MODE == 12
    qtr_lib_test_app_setup();
#elif APP_MODE == 13
    mini_messenger_test_app_setup();
#elif APP_MODE == 14
    motor_messenger_test_app_setup();
#elif APP_MODE == 15
    ir_test_updated_app_setup();
#elif APP_MODE == 16
    line_follow_test_app_setup();
#elif APP_MODE == 17
    motor_messenger_button_led_app_setup();
#else
#error "Unknown APP_MODE. Use 0=robot, 1=wifi test, 2=motor test, 3=upload test, 4=encoder test, 5=QTR test, 6=button/LED test, 7=simple Motoron test, 8=ultrasonic test, 9=servo test, 10=RFID test, 11=IR/QTR-RC test, 12=QTR library analog test, 13=MiniMessenger test, 14=motor MiniMessenger test, 15=updated IR/QTR-RC test, 16=line follow test, 17=motor MiniMessenger button/LED test."
#endif
}

void loop() {
#if APP_MODE == 0
    robot_app_loop();
#elif APP_MODE == 1
    wifi_test_app_loop();
#elif APP_MODE == 2
    motor_test_app_loop();
#elif APP_MODE == 3
    upload_test_app_loop();
#elif APP_MODE == 4
    encoder_test_app_loop();
#elif APP_MODE == 5
    qtr_test_app_loop();
#elif APP_MODE == 6
    button_led_test_app_loop();
#elif APP_MODE == 7
    simple_motoron_test_app_loop();
#elif APP_MODE == 8
    ultrasonic_test_app_loop();
#elif APP_MODE == 9
    servo_test_app_loop();
#elif APP_MODE == 10
    rfid_test_app_loop();
#elif APP_MODE == 11
    ir_test_app_loop();
#elif APP_MODE == 12
    qtr_lib_test_app_loop();
#elif APP_MODE == 13
    mini_messenger_test_app_loop();
#elif APP_MODE == 14
    motor_messenger_test_app_loop();
#elif APP_MODE == 15
    ir_test_updated_app_loop();
#elif APP_MODE == 16
    line_follow_test_app_loop();
#elif APP_MODE == 17
    motor_messenger_button_led_app_loop();
#endif
}
