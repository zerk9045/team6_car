#ifndef TEAM6_CAR_SERVO_H
#define TEAM6_CAR_SERVO_H

#include "pico/stdlib.h"

class Servo {
public:
    // Constructor
    Servo();
    // Destructor
    ~Servo();

    // Function to set the angle of the servo
    void setAngle(int angle);

    int getAngle();

private:
    int pin;        // Pin connected to the servo
    int minAngle;   // Minimum angle the servo can move to
    int maxAngle;   // Maximum angle the servo can move to
    int currAngle = 0;
};


#endif //TEAM6_CAR_SERVO_H
