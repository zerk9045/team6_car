#ifndef TEAM6_CAR_MOTOR_H
#define TEAM6_CAR_MOTOR_H

#include "pico/stdlib.h"
#include "IRSensor.h"
#include <string>

class Motor {
    IRSensor* irSensor;
public:
    // Constructor
    Motor();

    // Destructor
    ~Motor();
    static void set_pwm_pin(uint pin, uint freq, float duty_c);
    // Function to set motor speed
    void setSpeed(int speedPWM);
    // Function to read motor speed
    double getSpeed();
    void updateDirection(bool inAValue, bool inBValue, std::string direction);

private:
    int pwmPin; // PWM pin for motor speed control
    int inAPin; // INA pin for motor direction control
    int inBPin; // INB pin for motor direction control
    //IRSensor* irSensor; // IRSensor object for speed calculation
    // Private helper function to set motor direction
    std::string motor_direction;
    int currentPwm;
};

#endif //TEAM6_CAR_MOTOR_H