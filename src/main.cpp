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

void setup() {
#if APP_MODE == 0
    robot_app_setup();
#elif APP_MODE == 1
    wifi_test_app_setup();
#elif APP_MODE == 2
    motor_test_app_setup();
#else
#error "Unknown APP_MODE. Use 0=robot, 1=wifi test, 2=motor test."
#endif
}

void loop() {
#if APP_MODE == 0
    robot_app_loop();
#elif APP_MODE == 1
    wifi_test_app_loop();
#elif APP_MODE == 2
    motor_test_app_loop();
#endif
}
