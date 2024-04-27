#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "IRSensor.h"



Motor::Motor()
        : pwmPin(MOTOR_PWM), inAPin(INA_PIN), inBPin(INB_PIN), irSensor(new IRSensor()) {
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
    if (speedPWM > MAX_PWM) {
        speedPWM = MAX_PWM;
    } else if (speedPWM < MIN_PWM) {
        speedPWM = MIN_PWM;
    }

    // Set motor direction based on pwm signal
    if (speedPWM == BRAKE_PWM) {
        updateDirection(false, false); // INA high, INB high (brake)
    } else if (speedPWM > BRAKE_PWM) {
        updateDirection(true, false); // INA high, INB low (forward)
    } else {
        updateDirection(false, true); // INA low, INB high (reverse)
    }

    // Set PWM duty cycle for motor speed control pin
    pwm_set_gpio_level(pwmPin, speedPWM);
}

int Motor::getSpeed() {
    // Use the IRSensor to calculate the rotations per second
    return irSensor->getSpeed();
}

void Motor::updateDirection(bool inAValue, bool inBValue) {
    gpio_put(inAPin, inAValue ? 1 : 0);
    gpio_put(inBPin, inBValue ? 1 : 0);
}
