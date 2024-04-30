#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
//#include "IRSensor.h"

// Need to change this logic since max_pwm is the same for forward and reverse
#define BRAKE_PWM 1500000
#define MAX_PWM 3000000
#define MIN_PWM 1375000

Motor::Motor(){//, irSensor(new IRSensor()) {
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

//    // Set motor direction based on pwm signal
//    if (speedPWM == BRAKE_PWM) {
//        updateDirection(false, false); // INA high, INB high (brake)
//    } else if (speedPWM > BRAKE_PWM) {
//        updateDirection(true, false); // INA high, INB low (forward)
//    } else {
//        updateDirection(false, true); // INA low, INB high (reverse)
//    }

    if (currentPwm == speedPWM) {
        return;
    }
    //set_pwm_pin(MOTOR_PWM, 100, speedPWM/1000);
    pwm_set_gpio_level(MOTOR_PWM, (uint)(speedPWM/1000));
    currentPwm = speedPWM;
}

//int Motor::getSpeed() {
//    // Use the IRSensor to calculate the rotations per second
//    return irSensor->getSpeed();
//}

void Motor::updateDirection(bool inAValue, bool inBValue) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
}
