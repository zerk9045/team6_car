#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header

Motor::Motor()
        : pwmPin(MOTOR_PWM), inAPin(INA_PIN), inBPin(INB_PIN) {
    // Initialize motor hardware or perform any necessary setup here
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
    gpio_set_function(inAPin, GPIO_FUNC_OUTPUT);
    gpio_set_function(inBPin, GPIO_FUNC_OUTPUT);
}

Motor::~Motor() {
    // Cleanup resources if necessary
}

void Motor::setSpeed(int speed) {
    // Ensure speed is within the valid range
    if (!(speed > -100 && speed < 100)){
        return
    }

    // Set motor direction based on speed sign
    if (speed >= 0) {
        setDirection(true); // Forward direction
    } else {
        setDirection(false); // Reverse direction
        speed = -speed; // Convert negative speed to positive for PWM duty cycle
    }

    // Convert speed to PWM duty cycle (0 to 100%)
    float dutyCycle = (speed / 100.0f) * 100.0f;

    // Set PWM duty cycle for motor speed control pin
    pwm_set_gpio_level(pwmPin, dutyCycle);
}

void Motor::setDirection(bool forward) {
    // Set motor direction based on the forward flag
    if (forward) {
        updateDirection(true, false); // INA high, INB low (forward)
    } else {
        updateDirection(false, true); // INA low, INB high (reverse)
    }
}

void Motor::updateDirection(bool inAValue, bool inBValue) {
    gpio_put(inAPin, inAValue ? 1 : 0);
    gpio_put(inBPin, inBValue ? 1 : 0);
}
