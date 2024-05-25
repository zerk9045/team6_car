#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>

#define WHEEL_DIAMETER 0.05
#define M_PI        3.14159265358979323846264338327950288
#define MAX_DUTY    3000
#define MIN_DUTY    700

Motor::Motor(){//, ) {
    irSensor = new IRSensor();
    // Initialize motor hardware or perform any necessary setup here
    gpio_init(MOTOR_PWM);
    gpio_init(INA_PIN);
    gpio_init(INB_PIN);
    set_pwm_pin(MOTOR_PWM, 1000, MIN_DUTY);
    gpio_set_dir(INA_PIN, GPIO_OUT);
    gpio_set_dir(INB_PIN, GPIO_OUT);
    previous_error = 0;
    integral_error = 0;
    currentPwm = MIN_DUTY;
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

int Motor::getCurrentPwm(){
    return currentPwm;
}

void Motor::setSpeed(double speedPWM) {
    if (speedPWM > MAX_DUTY) {
        speedPWM = MAX_DUTY;
    } else if (speedPWM < MIN_DUTY) {
        speedPWM = MIN_DUTY;
    }
    // Ensure PWM is within the valid range
    if (currentPwm == speedPWM) {
        return;
    }
    pwm_set_gpio_level(MOTOR_PWM, (uint)(speedPWM));
    currentPwm = speedPWM;
}
std::string Motor::getDirection() {
    return motor_direction;
}

int Motor::getCount(){
    return irSensor->getCountsPerTimer();
}
double Motor::getSpeed() {
    double speed;
    if (motor_direction == "forward"){
        speed = static_cast<double>(
                ((irSensor->getCountsPerTimer()/0.5) * (2*M_PI)) * 0.05);
        irSensor->resetCounts();
    }
    else if (motor_direction == "reverse"){
        speed = static_cast<double>(
                -1* ((irSensor->getCountsPerTimer()/0.5) * (2*M_PI)) * 0.05);
        irSensor->resetCounts();
    }
    else {
        speed = 0;
        irSensor->resetCounts();
    }

    // Update the speed buffer with the new speed measurement
    speedBuffer[bufferIndex] = speed;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Calculate the average speed
    double averageSpeed = 0.0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        averageSpeed += speedBuffer[i];
    }
    averageSpeed /= BUFFER_SIZE;

    return speed;
//    //use an average filter to smooth out the speed measurements
//
//    // angular speed in rads/sec = (Revs per second / second) * (2pi)
//    // w = (irSensor->getCountsPerTimer()/0.3) * (2*M_PI);
//
//    // linear speed = angular speed * radius
//    // v = w * 0.05;
//
//    if (motor_direction == "forward"){
//        return static_cast<double>(
//                ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
//    }
//    else if (motor_direction == "reverse"){
//        return static_cast<double>(
//            -1* ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
//    }
//    else {
//        return 0;
//    }

}

void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
