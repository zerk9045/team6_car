#ifndef TEAM6_CAR_MOTOR_H
#define TEAM6_CAR_MOTOR_H

#include "pico/stdlib.h"
#include "IRSensor.h"
#define BRAKE_PWM 1500000;
#define MAX_PWM 1625000;
#define MIN_PWM 1375000;

class Motor {
public:
    // Constructor
    Motor();

    // Destructor
    ~Motor();

    // Function to set motor speed
    void setSpeed(int speedPWM);
    // Function to read motor speed
    int getSpeed();

private:
    int pwmPin; // PWM pin for motor speed control
    int inAPin; // INA pin for motor direction control
    int inBPin; // INB pin for motor direction control
    IRSensor* irSensor; // IRSensor object for speed calculation
    // Private helper function to set motor direction
    void updateDirection(bool inAValue, bool inBValue);
};

#endif //TEAM6_CAR_MOTOR_H