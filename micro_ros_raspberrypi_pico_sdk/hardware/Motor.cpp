#include "Motor.h"
#include "pico/stdlib.h"
#include "../config/pin_config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <string>

#define WHEEL_RADIUS 0.05
#define M_PI        3.14159265358979323846264338327950288
#define MAX_DUTY    77
#define MIN_DUTY    0

// Define timestamp for previousTime
absolute_time_t previousTime;

// Intialize variables
double circumference = 2*M_PI*WHEEL_RADIUS;
int previous_counts_per_timer = 0;
int counts_per_timer = 0;
static int count = 0;
double previous_cps = 0;
// Interrupt handler for the timer
absolute_time_t last_interrupt_time2;  

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
    return currCount;
}

// Function for getting the current speed of the motor
// It converts the IR sensor counts to speed in m/s
// The speed is calculated by counting the number of counts in a certain interval

double Motor::getSpeed() {
    count = 0;
    // Get the current time
    absolute_time_t currentTime = get_absolute_time();
    count = counts_per_timer;
    // Calculate the time difference
    int64_t deltaTimeMicro = absolute_time_diff_us(previousTime, currentTime);    

    // Convert from microseconds to seconds
    double deltaTime = static_cast<double>(deltaTimeMicro) / 1000000;
   
    // Calculate counts per second
    double cps = (count-previous_counts_per_timer)/ deltaTime;
    //irSensor->resetCounts();
    if (currCount == previous_counts_per_timer && motor_direction != "stop"){
    	cps = previous_cps;
    }
    previous_cps = cps;
    previous_counts_per_timer = count;

    // There are 3 white dots on the motor gear box. Theoretically, one wheel rotation
    // should equal 3 count. However, empircally measuring a single wheel rotation, 
    // there are 4 counts per wheel rotation.

    // Convert counts per second to revolutions per minute
    double rps = cps / 4;
    double rpm = rps * 60;

    // Set current time to previous time for next iteration
    previousTime = currentTime;

    // Calculate circumference
    double speed;

    if (motor_direction == "reverse"){
        speed = static_cast<double>(-1*rps*circumference);
    }
    else if (motor_direction == "forward"){
        speed = static_cast<double>(rps*circumference);
    }else if (motor_direction == "stop"){
        speed = 0.0;
         // Reset the counts per timer
         //irSensor->resetCounts();
    }

    // // Update the speed buffer with the new speed measurement
    // speedBuffer[bufferIndex] = speed;
    // bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // // Calculate the average speed
    // double averageSpeed = 0.0;
    // for (int i = 0; i < BUFFER_SIZE; i++) {
    //     averageSpeed += speedBuffer[i];
    // }
    // averageSpeed /= BUFFER_SIZE;

    return speed;
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

}

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
    count = counts_per_timer;
    // Calculate the time difference since the last interrupt
   int64_t diff_us = absolute_time_diff_us(last_interrupt_time2, now);

    // Check if the current rising edge is the actual rising edge or if it is part of the bouncing
    // If the time difference is greater than 8 ms, it is a valid rising edge
    if (diff_us > 800 && gpio_get(IR_SENSOR_PIN)){

        // Set the timestamp of this interrupt as the previous interrupt time for the next iteration
        last_interrupt_time2 = now;

        // Increment the number of counnts
        counts_per_timer = counts_per_timer + 1;
        
    }
}
