#ifndef TEAM6_CAR_MOTOR_H
#define TEAM6_CAR_MOTOR_H

#include "pico/stdlib.h"
#include "IRSensor.h"
#include <string>


#define BRAKE_PWM 1500000
#define MAX_PWM 2000000
#define MIN_PWM 1375000
#define BUFFER_SIZE 1

class Motor {
    IRSensor* irSensor;
    double speedBuffer[BUFFER_SIZE];
    int bufferIndex;
public:
    // Constructor
    Motor();
    double previous_error;
    double integral_error;
    absolute_time_t previous_time; // Previous time for speed calculation
    // Destructor
    ~Motor();
    static void set_pwm_pin(uint pin, uint freq, float duty_c);
    // Function to set motor speed
    void setSpeed(double speedPWM);

    void updateDirection(bool inAValue, bool inBValue, std::string direction);
    std::string getDirection();
    int getCurrentPwm();
    int getCount();
std::string motor_direction;
    // Stuff from IRSensor
    static void do_interrupt(uint gpio, uint32_t events);
    int getCountsPerTimer();

private:
    int pwmPin; // PWM pin for motor speed control
    int inAPin; // INA pin for motor direction control
    int inBPin; // INB pin for motor direction control
    //IRSensor* irSensor; // IRSensor object for speed calculation
    // Private helper function to set motor direction
    
    double currentPwm;
    double previous_speed_estimate; // Previous speed estimate for Kalman filter
    double estimated_error; // Estimated error for Kalman filter

};

#endif //TEAM6_CAR_MOTOR_H
