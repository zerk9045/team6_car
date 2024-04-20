#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"

#define MAX_PULSE_WIDTH 2
#define MIN_PULSE_WIDTH 1
#define PWM_PERIOD_MS 20

Servo::Servo(){
    // Initialize servo hardware or perform any necessary setup here
    gpio_set_function(SERVO_PWM, GPIO_FUNC_PWM);
}

Servo::~Servo() {
}

void Servo::setAngle(int angle) {
    // Ensure angle is within the specified range
    if (!(angle > 0 && angle < 180)){
        return;
    }


    currAngle = angle;
    // Calculate pulse width based on angle
    float pulseWidth = ((angle / 180.0f) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) + MIN_PULSE_WIDTH;

    // Convert pulse width to duty cycle
    float dutyCycle = (pulseWidth / PWM_PERIOD_MS) * 100.0f;

    // Set PWM duty cycle for servo control pin
    pwm_set_gpio_level(SERVO_PWM, dutyCycle);
}

int Servo::getAngle() {
    return currAngle;
}