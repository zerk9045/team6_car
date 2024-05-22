#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>
#include <numeric>

#define WHEEL_DIAMETER 0.05
#define M_PI        3.14159265358979323846264338327950288
#define MAX_SPEED 15.0
#define MIN_SPEED -15.0

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

void Motor::adjustSpeed(double adjustment) {
    // Calculate the new desired speed
    double newSpeed = currentSpeed + adjustment;

    // Ensure the new speed is within the valid range
    if (newSpeed > MAX_SPEED) {
        newSpeed = MAX_SPEED;
    } else if (newSpeed < MIN_SPEED) {
        newSpeed = MIN_SPEED;
    }

    // Set the new speed
    setSpeed(newSpeed);
}

void Motor::setSpeed(double speed) {
    std::string newDirection;
//    if (speed > 0) {
//        newDirection = "forward";
//    } else if (speed < 0) {
//        newDirection = "reverse";
//    } else {
//        newDirection = "stop";
//    }
    newDirection = "forward";
    // Only update direction if it has changed
    if (newDirection != motor_direction) {
        updateDirection(newDirection == "forward", newDirection == "reverse", newDirection);
    }
    // Convert the speed to a PWM value
    int speedPWM = speed;//mapSpeedToPwm(abs(speed));

    // Ensure PWM is within the valid range
    if (speedPWM > MAX_PWM) {
        speedPWM = MAX_PWM;
    } else if (speedPWM < MIN_PWM) {
        speedPWM = MIN_PWM;
    }
    if (currentPwm == speedPWM) {
        return;
    }
    pwm_set_gpio_level(MOTOR_PWM, (uint)(speedPWM / 1000.0)); // Assuming PWM range is 0-65535
    currentPwm = speedPWM;
}

int Motor::mapSpeedToPwm(double speed) {
    // Map the speed to a PWM value
    // 0.0 =
    int pwm = (speed - MIN_SPEED) * (MAX_PWM - MIN_PWM) / (MAX_SPEED - MIN_SPEED) + MIN_PWM;

    return pwm;
}

void Motor::pidController(double desiredSpeed) {
    // Constants
    double Kp = 0.1;
    double Ki = 0.01;
    double Kd = 0.01;

    // Variables
    static double integral = 0;
    static double last_error = 0;

    // The current speed is now stored in the private variable
    double error = desiredSpeed - currentSpeed;

    // Calculate the integral
    integral += error;

    // Calculate the derivative
    double derivative = error - last_error;

    // Calculate the output
    double output = Kp * error + Ki * integral + Kd * derivative;

    // Update the last error
    last_error = error;

    // Adjust the motor speed based on the PID output
    adjustSpeed(output);
}

std::string Motor::getDirection() {
    return motor_direction;
}

double Motor::getSpeed() {
    // Measure the speed
    double speed = 0; // Replace this with the actual code to measure the speed
    if (motor_direction == "forward"){
        speed = static_cast<double>(
                ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
    }
    else if (motor_direction == "reverse"){
        speed = static_cast<double>(
                -1* ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
    }

    // Add the new measurement to the list
    speedMeasurements.push_back(speed);

    // If we have more than N measurements, remove the oldest one
    if (speedMeasurements.size() > N) {
        speedMeasurements.pop_front();
    }

    // Calculate the average speed
    double sum = std::accumulate(speedMeasurements.begin(), speedMeasurements.end(), 0.0);
    currentSpeed = sum / speedMeasurements.size(); // Set currentSpeed

    return currentSpeed;
}

void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
