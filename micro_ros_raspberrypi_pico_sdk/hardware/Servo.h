#ifndef TEAM6_CAR_SERVO_H
#define TEAM6_CAR_SERVO_H

#include "pico/stdlib.h"
#define STRAIGHT_ANGLE_PWM 1500000
#define MAX_ANGLE_PWM 1625000
#define MIN_ANGLE_PWM 1375000

class Servo {
public:
    // Constructor
    Servo();
    // Destructor
    ~Servo();

    // Function to set the angle of the servo
    void setAngle(int anglePWM);

    int getAngle();

    void set_pwm_pin(int pin, uint freq, uint duty_c);
private:
    int pin;        // Pin connected to the servo
    int minAngle;   // Minimum angle the servo can move to
    int maxAngle;   // Maximum angle the servo can move to
    int currAnglePWM = 0;
};


#endif //TEAM6_CAR_SERVO_H
