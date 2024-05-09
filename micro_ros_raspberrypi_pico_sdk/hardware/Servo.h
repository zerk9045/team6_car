#ifndef TEAM6_CAR_SERVO_H
#define TEAM6_CAR_SERVO_H

#include "pico/stdlib.h"
#define STRAIGHT_SERVO_ANGLE_PWM 1500000
#define MAX_SERVO_ANGLE_PWM 2000000
#define MIN_SERVO_ANGLE_PWM 1000000

class Servo {
public:
    // Constructor
    Servo();
    // Destructor
    ~Servo();

    static void set_pwm_pin(uint pin, uint freq, float duty_c);
    // Function to set the angle of the servo
    void setAngle(int anglePWM);

    double getAngle();

private:
    int pin;        // Pin connected to the servo
    int minAngle;   // Minimum angle the servo can move to
    int maxAngle;   // Maximum angle the servo can move to
    int currAnglePWM = 0;
};


#endif //TEAM6_CAR_SERVO_H
