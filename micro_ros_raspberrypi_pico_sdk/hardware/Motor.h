#ifndef TEAM6_CAR_MOTOR_H
#define TEAM6_CAR_MOTOR_H

#include "pico/stdlib.h"

class Motor {
public:
    // Constructor
    Motor();

    // Destructor
    ~Motor();

    // Function to set motor speed (-100 to 100)
    void setSpeed(int speed);

    // Function to set motor direction (true for forward, false for reverse)
    void setDirection(bool forward);

private:
    int pwmPin; // PWM pin for motor speed control
    int inAPin; // INA pin for motor direction control
    int inBPin; // INB pin for motor direction control

    // Private helper function to set motor direction
    void updateDirection(bool inAValue, bool inBValue);
};

#endif //TEAM6_CAR_MOTOR_H