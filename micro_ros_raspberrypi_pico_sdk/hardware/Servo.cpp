#include "Servo.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header

#define MAX_PULSE_WIDTH 2
#define MIN_PULSE_WIDTH 1
#define PWM_PERIOD_MS 20

Servo::Servo()
        : pwmPin(SERVO_PWM), minAngle(0), maxAngle(180) {
    // Initialize servo hardware or perform any necessary setup here
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
}

Servo::~Servo() {
    // Disable PWM output for the servo pin
    pwm_set_enabled(pwmPin, false);
}

void Servo::setAngle(int angle) {
    // Ensure angle is within the specified range
    angle = std::clamp(angle, minAngle, maxAngle);

    currAngle = angle;
    // Calculate pulse width based on angle
    float pulseWidth = ((angle / 180.0f) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) + MIN_PULSE_WIDTH;

    // Convert pulse width to duty cycle
    float dutyCycle = (pulseWidth / PWM_PERIOD_MS) * 100.0f;

    // Set PWM duty cycle for servo control pin
    pwm_set_gpio_level(pwmPin, dutyCycle);
}

int Servo::getAngle() {
    return currAngle;
}