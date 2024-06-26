#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include <string>

#define WHEEL_RADIUS 0.05
#define M_PI        3.14159265358979323846264338327950288
<<<<<<< HEAD
#define MAX_DUTY    77
#define MIN_DUTY    0

// Define timestamp for previousTime
absolute_time_t previousTime;

// Intialize variables
int previous_counts_per_timer = 0;
int counts_per_timer = 0;
int currentCount = 0;
double previous_cps = 0;
bool isCounting =false;
// Interrupt handler for the timer
absolute_time_t last_interrupt_time2;  
=======
#define MAX_DUTY    1100
#define MIN_DUTY    1000
>>>>>>> my-temp-work

Motor::Motor(){//, ) {
    //irSensor = new IRSensor();
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(IR_SENSOR_PIN, GPIO_IRQ_EDGE_FALL, true, Motor::do_interrupt);
    // Initialize motor hardware or perform any necessary setup here
    gpio_init(MOTOR_PWM);
    gpio_init(INA_PIN);
    gpio_init(INB_PIN);
    set_pwm_pin(MOTOR_PWM, 1000, MIN_DUTY);
    gpio_set_dir(INA_PIN, GPIO_OUT);
    gpio_set_dir(INB_PIN, GPIO_OUT);
    currentPwm = MIN_DUTY;
    previous_time = get_absolute_time();
}

Motor::~Motor() {
    // Cleanup resources if necessary
    //delete irSensor;
}

// Function for setting the PWM settings of the selected GPIO pin
void Motor::set_pwm_pin(uint pin, uint freq, float duty_c) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (freq * 255);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 255);
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(pin, (uint)(duty_c)); //connect the pin to the pwm engine and set the on/off level.
}

// Function for getting the current PWM of the motor
int Motor::getCurrentPwm(){
    return currentPwm;
}

// Function for setting the current speed of the motor in PWM
void Motor::setSpeed(double speedPWM) {
    if (speedPWM > MAX_DUTY) {
        speedPWM = MAX_DUTY;
    } else if (speedPWM < MIN_DUTY) {
        speedPWM = MIN_DUTY;
    }
    // Ensure PWM is within the valid range
    if (currentPwm == speedPWM) {
        return;
    }
    pwm_set_gpio_level(MOTOR_PWM, (uint)(speedPWM));
    currentPwm = speedPWM;
}

// Function for getting the direction for the motor
std::string Motor::getDirection() {
    return motor_direction;
}

// Function for getting the number of counts within a certain interval
int Motor::getCount(){
<<<<<<< HEAD
    return currentCount;
=======
    return irSensor->getCountsPerTimer();
}
double Motor::getSpeed() {

    u_int32_t old_irq = save_and_disable_interrupts();

    int cps = getCount() / 0.6;

    double speed;
    if (motor_direction == "forward"){
        speed = static_cast<double>(
                ((cps/4) * (2*M_PI)) * 0.05);
    }
    else if (motor_direction == "reverse"){
        speed = static_cast<double>(
                -1* ((cps/4) * (2*M_PI)) * 0.05);
    }
    else {
        speed = 0;
        irSensor->resetCounts();
    }

    

    absolute_time_t currentTime = get_absolute_time();
    int64_t deltaTimeMicro = absolute_time_diff_us(prevTime, currentTime);
    deltaTime = static_cast<double>(deltaTimeMicro) / 1000000;

    prevTime = currentTime;

    // Update the speed buffer with the new speed measurement
    speedBuffer[bufferIndex] = speed;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Calculate the average speed
    double averageSpeed = 0.0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        averageSpeed += speedBuffer[i];
    }
    averageSpeed /= BUFFER_SIZE;
    restore_interrupts(old_irq);
    return averageSpeed;
//    //use an average filter to smooth out the speed measurements
//
//    // angular speed in rads/sec = (Revs per second / second) * (2pi)
//    // w = (irSensor->getCountsPerTimer()/0.3) * (2*M_PI);
//
//    // linear speed = angular speed * radius
//    // v = w * 0.05;
//
//    if (motor_direction == "forward"){
//        return static_cast<double>(
//                ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
//    }
//    else if (motor_direction == "reverse"){
//        return static_cast<double>(
//            -1* ((irSensor->getCountsPerTimer()/0.1) * (2*M_PI)) * 0.05);
//    }
//    else {
//        return 0;
//    }

>>>>>>> my-temp-work
}

// Function for getting the current speed of the motor
// It converts the IR sensor counts to speed in m/s
// The speed is calculated by counting the number of counts in a certain interval


// Function for updating the direction of the motor
void Motor::updateDirection(bool inAValue, bool inBValue, std::string direction) {
    gpio_put(INA_PIN, inAValue ? 1 : 0);
    gpio_put(INB_PIN, inBValue ? 1 : 0);
    motor_direction = direction;
}

// readEncoder function
void Motor::do_interrupt(uint gpio, uint32_t events) {

    // Debouncing logic
    // Get the current time
    absolute_time_t now = get_absolute_time();
    isCounting = true;
    // Calculate the time difference since the last interrupt
   int64_t diff_us = absolute_time_diff_us(last_interrupt_time2, now);

    // Check if the current rising edge is the actual rising edge or if it is part of the bouncing
    // If the time difference is greater than 8 ms, it is a valid rising edge
    if (diff_us > 800 && gpio_get(IR_SENSOR_PIN)){

        // Set the timestamp of this interrupt as the previous interrupt time for the next iteration
        last_interrupt_time2 = now;

        // Increment the number of counnts
        counts_per_timer = counts_per_timer + 1;

        isCounting = false;
    }
   
}

int Motor::getCountsPerTimer(){
    return counts_per_timer;
}
bool Motor::getisCounting(){
    return isCounting;
}