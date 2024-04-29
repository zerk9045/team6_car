#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#define STRAIGHT_ANGLE_PWM 1500000
#define MAX_ANGLE_PWM 1625000
#define MIN_ANGLE_PWM 1375000

Servo::Servo(){
    gpio_init(SERVO_PWM);
    set_pwm_pin(SERVO_PWM, 100, 0);
}

Servo::~Servo() {
}

void Servo::set_pwm_pin(uint pin, uint freq, uint duty_c) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000);
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, duty_c); //connect the pin to the pwm engine and set the on/off level.
}

void Servo::setAngle(int anglePWM) {
    // Ensure pwm is within the specified range
    if (anglePWM > MAX_ANGLE_PWM) {
        anglePWM = MAX_ANGLE_PWM;
    } else if (anglePWM < MIN_ANGLE_PWM) {
        anglePWM = MIN_ANGLE_PWM;
    }

    // Set PWM duty cycle for servo control pin//1500000

    set_pwm_pin(SERVO_PWM, 100, anglePWM/1000000000);

    currAnglePWM = anglePWM;
}



int Servo::getAngle() {
    // convert the pwm signal to an angle
//    int rangePWM = MAX_ANGLE_PWM - MIN_ANGLE_PWM;
//    float scale = 2.0 / rangePWM; // Adjust the scale to map to the range -1 to 1
//    int normalizedAngle = ((currAnglePWM - MIN_ANGLE_PWM) * scale) - 1; // Subtract 1 to shift the range to -1 to 1

    return currAnglePWM;
}