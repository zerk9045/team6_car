#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <cstdlib>

#define M_PI        3.14159265358979323846264338327950288
#define THRESHOLD 5000
Servo::Servo(){
    gpio_init(SERVO_PWM);
    set_pwm_pin(SERVO_PWM, 100, STRAIGHT_SERVO_ANGLE_PWM/1000);
}

Servo::~Servo() {
}

void Servo::set_pwm_pin(uint pin, uint freq, float duty_c) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config1 = pwm_get_default_config();
    float div1 = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config1, div1);
    pwm_config_set_wrap(&config1, 10000);
    pwm_init(slice_num, &config1, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, (uint)(duty_c)); //connect the pin to the pwm engine and set the on/off level.
}

void Servo::setAngle(int anglePWM) {
    // Ensure pwm is within the specified range
    if (anglePWM > MAX_SERVO_ANGLE_PWM) {
        anglePWM = MAX_SERVO_ANGLE_PWM;
    } else if (anglePWM < MIN_SERVO_ANGLE_PWM) {
        anglePWM = MIN_SERVO_ANGLE_PWM;
    }

    // Set PWM duty cycle for servo control pin//1500000
    // Convert anglePWM to duty cycle percentage (assuming MAX_ANGLE_PWM is the maximum PWM value)
    float duty_cycle = (float)anglePWM / MAX_SERVO_ANGLE_PWM;
    if (currAnglePWM == anglePWM) {
        return;
    }
    pwm_set_gpio_level(SERVO_PWM, (uint)(anglePWM/1000));
    currAnglePWM = anglePWM;
}



double Servo::getAngle() {
    // Normalize PWM to the range of -π/2 to π/2 radians
    double angleRadians = ((double)currAnglePWM - STRAIGHT_SERVO_ANGLE_PWM) / (MAX_SERVO_ANGLE_PWM - MIN_SERVO_ANGLE_PWM) * M_PI / 2;

    return angleRadians;
}