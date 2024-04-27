#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"



Servo::Servo(){
    // Initialize servo hardware or perform any necessary setup here
    gpio_set_function(SERVO_PWM, GPIO_FUNC_PWM);
}

Servo::~Servo() {
}

void Servo::setAngle(int anglePWM) {
    // Ensure pwm is within the specified range
    if (anglePWM > MAX_ANGLE_PWM) {
        anglePWM = MAX_ANGLE_PWM;
    } else if (anglePWM < MIN_ANGLE_PWM) {
        anglePWM = MIN_ANGLE_PWM;
    }

    // Set PWM duty cycle for servo control pin
    pwm_set_gpio_level(SERVO_PWM, anglePWM);

    currAnglePWM = anglePWM;
}

int Servo::getAngle() {
    // convert the pwm signal to an angle
    int rangePWM = MAX_ANGLE_PWM - MIN_ANGLE_PWM;
    float scale = 2.0 / rangePWM; // Adjust the scale to map to the range -1 to 1
    int normalizedAngle = ((currAnglePWM - MIN_ANGLE_PWM) * scale) - 1; // Subtract 1 to shift the range to -1 to 1

    return normalizedAngle;
}