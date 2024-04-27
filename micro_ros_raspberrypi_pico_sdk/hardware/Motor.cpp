#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h" // Include the pin configuration header
#include "hardware/pwm.h"
#define BRAKEPWM = 1500000;
#define MAXPWM = 1625000;
#define MINPWM = 1375000;

Motor::Motor()
        : pwmPin(MOTOR_PWM), inAPin(INA_PIN), inBPin(INB_PIN) {
    // Initialize motor hardware or perform any necessary setup here
    gpio_set_function(pwmPin, GPIO_FUNC_PWM);
    gpio_set_dir(inAPin, GPIO_OUT);
    gpio_set_dir(inBPin, GPIO_OUT);
}

Motor::~Motor() {
    // Cleanup resources if necessary
}

void Motor::setSpeed(int speedPWM) {
    // Ensure PWM is within the valid range
    if (speedPWM > MAXPWM) {
        speedPWM = MAXPWM;
    } else if (speedPWM < MINPWM) {
        speedPWM = MINPWM;
    }

    // Set motor direction based on pwm signal
    if (speedPWM == BRAKEPWM) {
        updateDirection(false, false); // INA high, INB high (brake)
    } else if (speedPWM > BRAKEPWM) {
        updateDirection(true, false); // INA high, INB low (forward)
    } else {
        updateDirection(false, true); // INA low, INB high (reverse)
    }

    // Set PWM duty cycle for motor speed control pin
    pwm_set_gpio_level(pwmPin, speedPWM);
}

void Motor::updateDirection(bool inAValue, bool inBValue) {
    gpio_put(inAPin, inAValue ? 1 : 0);
    gpio_put(inBPin, inBValue ? 1 : 0);
}
