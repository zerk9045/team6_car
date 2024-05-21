#ifndef TEAM6_CAR_MOTOR_H
#define TEAM6_CAR_MOTOR_H

#include "pico/stdlib.h"
#include "IRSensor.h"
#include <string>
#include <deque>

#define BRAKE_PWM 1500000
#define MAX_PWM 3000000
#define MIN_PWM 1375000

class Motor {
    IRSensor* irSensor;
    std::deque<double> speedMeasurements; // Store the last N speed measurements
    static const int N = 10; // Number of measurements to average
public:
    // Constructor
    Motor();

    // Destructor
    ~Motor();
    static void set_pwm_pin(uint pin, uint freq, float duty_c);
    // Function to set motor speed
    void setSpeed(double desiredSpeed);
    // Function to read motor speed
    double getSpeed();
    void updateDirection(bool inAValue, bool inBValue, std::string direction);
    std::string getDirection();
    int mapSpeedToPwm(double speed);
    void pidController(double desiredSpeed);

private:
    int pwmPin; // PWM pin for motor speed control
    int inAPin; // INA pin for motor direction control
    int inBPin; // INB pin for motor direction control
    //IRSensor* irSensor; // IRSensor object for speed calculation
    // Private helper function to set motor direction
    std::string motor_direction;
    int currentPwm;
    double previous_speed_estimate; // Previous speed estimate for Kalman filter
    double estimated_error; // Estimated error for Kalman filter
};

#endif //TEAM6_CAR_MOTOR_H