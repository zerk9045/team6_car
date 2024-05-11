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

    // Initialize the Kalman filter variables
    previous_speed_estimate = 0.0; // Initial speed estimate, can be set to 0
    estimated_error = 1.0; // Initial error estimate, can be set to a high value
}

double Motor::getSpeed() {
    //Linear Speed (m/s)=RPS×Circumference
    //Gear Ratio = (# Spur Gear Teeth /# Pinion Gear Teeth )x 2.72
    //    double rps = ; // Divide by 3 since we get 3 interrupts per revolution
    //    double tireRPS = rps/2.72; // Divide by gear ratio to get the tire RPS
    //cpr = 19.4778744511


// Old speed calculations
//    if(motor_direction == "forward"){
//        return static_cast<double>((irSensor->getCountsPerTimer()/0.3)*0.1*3.14159265358979323846264338327950288);//static_cast<double>(2*(irSensor->getCountsPerTimer()/19.4778744511)*0.15707963267);///0.10)*0.05*3.14);///3/2.72 * 0.05 * 3.14;
//    }
//    else if(motor_direction == "reverse"){
//        return static_cast<double>(-1*(irSensor->getCountsPerTimer()/0.3)*0.1*3.14159265358979323846264338327950288);//static_cast<double>(-1*2*(irSensor->getCountsPerTimer()/19.4778744511)*0.15707963267);///0.10)*0.05*3.14);///3/2.72 * 0.05 * 3.14;
//    }
//    else{
//        return 0;
//    }
// return static_cast<double>(irSensor->getCountsPerTimer());

// testing out different way of calculating speed
    
    // angular speed in rads/sec = (Revs per second / second) * (2pi)
    // w = (irSensor->getCountsPerTimer()/0.3) * (2*M_PI);

    // linear speed = angular speed * radius
    // v = w * 0.05;

    if (motor_direction == "forward"){
        return static_cast<double>(
            ((irSensor->getCountsPerTimer()/0.3) * (2*M_PI)) * 0.05;
        )
    }
    else if (motor_direction == "reverse"){
        return static_cast<double>(
            -1* ((irSensor->getCountsPerTimer()/0.3) * (2*M_PI)) * 0.05;
        )
    }
    else {
        return 0;
    }

return static_cast<double>(irSensor->getCountsPerTimer());


// Commenting out the Kalman Filter for now
    // Get the raw speed reading
//     double raw_speed = static_cast<double>((irSensor->getCountsPerTimer()/0.3)*0.1*3.14159265358979323846264338327950288);

//     // Define the system model parameters
//     double process_noise = 0.05; // This would depend on your specific system
//     double sensor_noise = 0.05; // This would depend on your specific sensor

//     // Perform the Kalman filter update
//     double kalman_gain = estimated_error / (estimated_error + sensor_noise);
//     double current_speed_estimate = previous_speed_estimate + kalman_gain * (raw_speed - previous_speed_estimate);
//     estimated_error = (1.0 - kalman_gain) * estimated_error + fabs(previous_speed_estimate - current_speed_estimate) * process_noise;

//     // Store the current speed estimate for the next iteration
//     previous_speed_estimate = current_speed_estimate;

//     // If the motor is moving in reverse, return the speed as a negative value
//     if (motor_direction == "reverse") {
//         current_speed_estimate = -current_speed_estimate;
//     }

//     // Return the speed estimate
//     return current_speed_estimate;
}

void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
