#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>

#define BRAKE_PWM 1500000
#define MAX_PWM 3000000
#define MIN_PWM 1375000
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
}

double Motor::getSpeed() {
    //Linear Speed (m/s)=RPS×Circumference
    //Gear Ratio = (# Spur Gear Teeth /# Pinion Gear Teeth )x 2.72
    //    double rps = ; // Divide by 3 since we get 3 interrupts per revolution
    //    double tireRPS = rps/2.72; // Divide by gear ratio to get the tire RPS
    //cpr = 19.4778744511

    // Constants
    // pi = 3.14159265358979323846264338327950288
    // wheel + tire diameter = REMEASURE
    // counts per revolution = 1

    // Formulas to find speed in m/s
    // Distance per count = (wheel diameter * pi) / (counts per revolution)
    // Speed = (Distance per count * Counts per second) / (counts per revolution)
    
//    if(motor_direction == "forward"){
//        return static_cast<double>(2*irSensor->getCountsPerTimer()*0.15707963267);//static_cast<double>(2*(irSensor->getCountsPerTimer()/19.4778744511)*0.15707963267);///0.10)*0.05*3.14);///3/2.72 * 0.05 * 3.14;
//    }
//    else if(motor_direction == "reverse"){
//        return static_cast<double>(-1*2*irSensor->getCountsPerTimer()*0.15707963267);//static_cast<double>(-1*2*(irSensor->getCountsPerTimer()/19.4778744511)*0.15707963267);///0.10)*0.05*3.14);///3/2.72 * 0.05 * 3.14;
//    }
//    else{
//        return 0;
//    }
    return static_cast<double>(irSensor->getCountsPerTimer());
}

void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
