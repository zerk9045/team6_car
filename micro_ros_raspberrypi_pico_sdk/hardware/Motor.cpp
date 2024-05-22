#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>

#define WHEEL_DIAMETER 0.05
#define M_PI        3.14159265358979323846264338327950288

Motor::Motor(){//, ) {
    irSensor = new IRSensor();
    // Initialize motor hardware or perform any necessary setup here
    gpio_init(MOTOR_PWM);
    gpio_init(INA_PIN);
    gpio_init(INB_PIN);
    set_pwm_pin(MOTOR_PWM, 100, 0);
    gpio_set_dir(INA_PIN, GPIO_OUT);
    gpio_set_dir(INB_PIN, GPIO_OUT);

}

Motor::~Motor() {
    // Cleanup resources if necessary
    delete irSensor;
}

void Motor::set_pwm_pin(uint pin, uint freq, float duty_c) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 10000);
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, (uint)(duty_c)); //connect the pin to the pwm engine and set the on/off level.
}

void Motor::setSpeed(int speedPWM) {
    // Ensure PWM is within the valid range
    if (speedPWM > MAX_PWM) {
        speedPWM = MAX_PWM;
    } else if (speedPWM < MIN_PWM) {
        speedPWM = MIN_PWM;
    }
    if (currentPwm == speedPWM) {
        return;
    }
    pwm_set_gpio_level(MOTOR_PWM, (uint)(speedPWM/1000));
    currentPwm = speedPWM;

    // Initialize the Kalman filter variables
    previous_speed_estimate = 0.0; // Initial speed estimate, can be set to 0
    estimated_error = 1.0; // Initial error estimate, can be set to a high value
}
std::string Motor::getDirection() {
    return motor_direction;
}


double Motor::getSpeed() {

    // angular speed in rads/sec = (Revs per second / second) * (2pi)
    // w = (irSensor->getCountsPerTimer()/0.3) * (2*M_PI);

    // linear speed = angular speed * radius
    // v = w * 0.05;

    if (motor_direction == "forward"){
        return static_cast<double>(
                ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
    }
    else if (motor_direction == "reverse"){
        return static_cast<double>(
            -1* ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
    }
    else {
        return 0;
    }

}

void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
