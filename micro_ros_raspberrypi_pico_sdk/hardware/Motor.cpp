#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>
#include <numeric>

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

void Motor::setSpeed(double desiredSpeed) {
    if (desiredSpeed > 0) {
        updateDirection(true, false, "forward");
    } else if (desiredSpeed < 0) {
        updateDirection(false, true, "reverse");
    } else {
        updateDirection(false, false, "stop");
    }
    // Convert the speed to a PWM value
    int speedPWM = mapSpeedToPwm(abs(desiredSpeed));

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

int Motor::mapSpeedToPwm(double speed) {
    // Define the maximum and minimum possible speeds
    double maxSpeed = 15.0; // Set this to the maximum possible speed
    double minSpeed = 1.0; // Set this to the minimum possible speed

    // Map the speed to a PWM value
    int pwm = (speed - minSpeed) * (MAX_PWM - MIN_PWM) / (maxSpeed - minSpeed) + MIN_PWM;

    return pwm;
}

void Motor::pidController(double desiredSpeed) {
    // Constants
    double Kp = 0.1;
    double Ki = 0.01;
    double Kd = 0.01;

    // Variables
    double currentSpeed = getSpeed();
    double error = desiredSpeed - currentSpeed;
    double integral = 0;
    double derivative = 0;
    double last_error = 0;
    double output = 0;

    // Calculate the integral
    integral += error;

    // Calculate the derivative
    derivative = error - last_error;

    // Calculate the output
    output = Kp * error + Ki * integral + Kd * derivative;

    // Update the last error
    last_error = error;

    // Set the motor speed
    setSpeed(output);
}

std::string Motor::getDirection() {
    return motor_direction;
}

double Motor::getSpeed() {
    // Measure the speed
    double speed = 0; // Replace this with the actual code to measure the speed
    // angular speed in rads/sec = (Revs per second / second) * (2pi)
    // w = (irSensor->getCountsPerTimer()/0.3) * (2*M_PI);

    // linear speed = angular speed * radius
    // v = w * 0.05;

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
    double averageSpeed = sum / speedMeasurements.size();

    return averageSpeed;
}



void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}
