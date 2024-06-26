#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>
#include "hardware/Motor.h"
#include <string>
#include <rmw_microros/rmw_microros.h>
#include <sstream>
#include <vector>
#include <iterator>
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include "../config/pin_config.h"
#include <fstream>
#include <chrono>
<<<<<<< HEAD
#include <cmath>
=======
>>>>>>> my-temp-work
#include "hardware/sync.h"
#define PID_LOGGING_ENABLED 1 // Use to enable or disable PID logging
#define KP_TEST 0 // Use to test different Kp values
double KP_GLOBAL = 0.01; // Proportional gain
std::chrono::time_point<std::chrono::system_clock> start_time;
// Declare a new publisher
rcl_publisher_t log_publisher;
rcl_publisher_t publisher;
rcl_subscription_t motor_subscriber;
rcl_subscription_t servo_subscriber;
std_msgs__msg__Float32 motor_msg;
std_msgs__msg__String control_msg;
std_msgs__msg__String log_msg;
std_msgs__msg__Int32 servo_msg;
Motor motor;
Servo servo;

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rcl_timer_t control_timer;
rcl_timer_t log_timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

absolute_time_t prevTime;
<<<<<<< HEAD
int currentCounts = 0;
int previousCounts = 0;
double circumference = 2*3.14159265358979323846264338327950288*0.05;
double previousCps = 0;
=======
>>>>>>> my-temp-work

double log_kp;
double log_error;
double log_integral_error;
double log_derivative;
double log_new_pwm;
double log_current_speed;
double log_desired_speed;
std::string log_dir;
void control_timer_callback(rcl_timer_t * control_timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (control_timer != NULL) {

        std_msgs__msg__String__init(&control_msg);
<<<<<<< HEAD
        std::string data = "1";//std::to_string(servo.getAngle()) + " " + std::to_string(log_current_speed);
        //std::string data = std::to_string(motor.getCount()) + " " + std::to_string(motor.getSpeed());
        
=======
        std::string data = std::to_string(servo.getAngle()) + " " + std::to_string(log_current_speed);
>>>>>>> my-temp-work
        control_msg.data.data = strdup(data.c_str()); // Create a copy of the string
        control_msg.data.size = strlen(control_msg.data.data);
        control_msg.data.capacity = control_msg.data.size + 1;
        rcl_ret_t ret = rcl_publish(&publisher, &control_msg, NULL);
        
        std_msgs__msg__String__fini(&control_msg);
        
}	
}
void log_timer_callback(rcl_timer_t * log_timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
    if (log_timer != NULL) {
    if(PID_LOGGING_ENABLED){
    std_msgs__msg__String__init(&log_msg);
        // Format the log data as a string
        std::stringstream log_data;
         log_data << /*timestamp << "," <<*/ log_kp << "," << log_integral_error << "," <<
         log_derivative << "," << log_new_pwm << "," << log_error << ","
         << log_current_speed << "," << log_desired_speed;
        
//        log_data << /*timestamp << "," <<*/ log_new_pwm << ","
//        << log_current_speed << "," << log_desired_speed << "," << log_dir;

        // Assign the log data to the message
        log_msg.data.data = strdup(log_data.str().c_str());
        log_msg.data.size = strlen(log_msg.data.data);
        log_msg.data.capacity = log_msg.data.size + 1;

        // Publish the message
        rcl_ret_t ret = rcl_publish(&log_publisher, &log_msg, NULL);
    }
        // Free the memory allocated for the message data
        //free(msg1.data.data);
        std_msgs__msg__String__fini(&log_msg);
    }
}

// TODO: add error checks to ensure the message is in the correct format
void subscription_callback_servo(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int pwm = msg->data;
    servo.setAngle(pwm);
}

bool isValidDirection(const std::string& direction) {
    return direction == "forward" || direction == "reverse" || direction == "stop";
}

bool isValidPwm(int pwm) {
    return pwm >= MIN_PWM && pwm <= MAX_PWM;
}

// TODO: add a custom motor message so we dont have to parse the string
//https://micro.ros.org/docs/tutorials/advanced/create_new_type/

void subscription_callback_motor(const void *msgin) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

    std::string direction;
    bool forward = false;
    bool reverse = false;

    // Temporarily disable interrupts
    uint32_t old_irq = save_and_disable_interrupts();

    // Grab the current counts
    currentCounts = motor.getCountsPerTimer();

    // Restore interrupts
    restore_interrupts(old_irq);


    // Calculate velocity
    // Get the current time
    absolute_time_t currentTime = get_absolute_time();

    // Calculate the time difference in microseconds
    int64_t deltaTimeMicro = absolute_time_diff_us(prevTime, currentTime);

    // Convert from microseconds to seconds
    double deltaT = static_cast<double>(deltaTimeMicro) / 1000000;

    // Calculate counts per second
     double cps = (currentCounts - previousCounts) / deltaT;

    // // Prevent 0s
    // if (cps == 0 && motor.motor_direction != "stop"){
    //     cps = previousCps;
    // }

    // Set current counts and time to previous counts and time for next iteration
    previousCounts = currentCounts;
    prevTime = currentTime;
    // if (cps !=0 && motor.motor_direction != "stop"){
    //     previousCps = cps;
    // } else if (motor.motor_direction == "stop"){
    //     previousCps = 0;
    // }
   
    

    // Calculate revs per second
    double rps = cps/4;
    double rpm = rps*60;

    // Calculate meters per second
    double current_speed;

    if (motor.motor_direction == "reverse"){
        current_speed = static_cast<double>(-1*rps*circumference);
    }
    else if (motor.motor_direction == "forward"){
        current_speed = static_cast<double>(rps*circumference);
    }else if (motor.motor_direction == "stop"){
        current_speed = 0.0;
    }

    // Grab the target speed
    double desired_speed =  static_cast<double>(msg->data);

    if (desired_speed > 0){
        forward = true;
        reverse = false;
        direction = "forward";
    }
    else if(desired_speed < 0){
        forward = false;
        reverse = true;
        direction = "reverse";
    }
    else{
<<<<<<< HEAD
        forward = false;
        reverse = false;
        direction = "stop";
=======
    	direction = "stop";
        motor.setSpeed(1000);
>>>>>>> my-temp-work
    }

    // Update the motor direction to the desired direction
    motor.updateDirection(reverse, forward, direction);
<<<<<<< HEAD

    // PID gains
    double Kp = 2.0;
    double Ki = 0.01;
    double Kd = 0.0;
=======
    double Kp = 1.5;
    // Use Ziegler–Nichols method to tune the PID controller
    /*1. Find Kmax the value of Kp where it begins to oscillate
      2. Measure the Tu period of the Kmax oscillation
      3. Set Kp = 0.6*Kmax, Ki = 2*Kp/Tu, Kd = Kp*(Tu/8)
    */
 
    double Ki = 0.00; // Integral gain, tweak this value
    double Kd = 0.1; // Derivative gain, tweak this value
    // Measure the current speed
    double current_speed = motor.getSpeed();
>>>>>>> my-temp-work

    // Calculate the error
    double error = desired_speed - current_speed;

<<<<<<< HEAD
    // Compute the integral term
    motor.integral_error += error * deltaT;

    // Compute the derivative term
    double derivative = (error - motor.previous_error) / deltaT;

    // Compute the output signal
    double u = Kp * error + Ki * motor.integral_error + Kd * derivative;

    // // Set new direction if depnding on the error sign
    // if (u > 0){
    //     forward = true;
    //     reverse = false;
    //     direction = "forward";
    // }
    // else if(u < 0){
    //     forward = false;
    //     reverse = true;
    //     direction = "reverse";
    // }
    // motor.updateDirection(reverse, forward, direction);
=======
    // Calculate the integral term
    motor.integral_error += error * motor.deltaTime;

    // Calculate the derivative term
    double derivative = (error - motor.previous_error) / motor.deltaTime;
   
>>>>>>> my-temp-work

    // Adjust the PWM based on the error
    double new_pwm = std::abs(motor.getCurrentPwm() + u);

    // Checks for PWM limits are in conducted in the following function
    // Set the new PWM value to the motor
    if (motor.motor_direction == "stop"){
        motor.setSpeed(0);
        currentCounts = 0;
    } else {
        motor.setSpeed(new_pwm);       
    }



    if(PID_LOGGING_ENABLED){
        log_kp = Kp;
        log_integral_error = motor.integral_error;
        log_derivative = cps;
        log_new_pwm = rpm;
        log_error = error;
        log_desired_speed = desired_speed;
        log_current_speed = current_speed;
        log_dir = motor.getDirection();
    }
}

bool pingAgent(){
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 100;
    const uint8_t attempts = 1;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
        gpio_put(LED_PIN, 0);
        return false;
    } else {
        gpio_put(LED_PIN, 1);
    }
    return true;
}

void createEntities(){
    allocator = rcl_get_default_allocator();

    // create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create rcl_node
    rclc_node_init_default(&node, "pico_node", "", &support);
    // create a timer,
    const unsigned int timer_timeout = 100;
    
    rclc_timer_init_default(
            &control_timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            control_timer_callback);
  rclc_timer_init_default(
    &log_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    log_timer_callback);  
    if(PID_LOGGING_ENABLED){
     // create log publisher
    rclc_publisher_init_default(
            &log_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "log_topic");
    }	
   
    // create a publisher
    rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "control_topic");
    // create a subscriber for the motor
    rclc_subscription_init_default(
            &motor_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "pi_motor_publishing_topic");

    //Create a subscriber for the servo
    rclc_subscription_init_default(
            &servo_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pi_servo_publishing_topic");

    // create executor
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_timer(&executor, &control_timer);
	rclc_executor_add_timer(&executor, &log_timer);
    // add subscribers
    rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &subscription_callback_motor, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &subscription_callback_servo, ON_NEW_DATA);

}

void destroyEntities(){
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_ret_t ret;

    // free resources
    ret = rcl_publisher_fini(&publisher, &node);
    ret = rcl_timer_fini(&log_timer);
    ret = rcl_timer_fini(&control_timer);
    ret = rcl_subscription_fini(&motor_subscriber, &node);
    ret = rcl_subscription_fini(&servo_subscriber, &node);
    ret = rcl_node_fini(&node);
    ret = rcl_publisher_fini(&log_publisher, &node);
}

int main()
{
    start_time = std::chrono::system_clock::now();
    stdio_init_all();
    rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
    );

    bool pulse;
    gpio_init(PULSE_LED);
    gpio_set_dir(PULSE_LED, GPIO_OUT);
    gpio_put(PULSE_LED, pulse);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);

    allocator = rcl_get_default_allocator();
    state = WAITING_AGENT;

    while (true){
        switch (state) {
            case WAITING_AGENT:
                state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
                break;
            case AGENT_AVAILABLE:
                createEntities();
                state = AGENT_CONNECTED ;
                break;
            case AGENT_CONNECTED:
                state = pingAgent() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin(&executor);
                }
                break;
            case AGENT_DISCONNECTED:
                destroyEntities();
                state = WAITING_AGENT;

                break;
            default:
                break;
        }
        pulse = ! pulse;
        gpio_put(PULSE_LED, pulse);
    }
    return 0;
}
