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
#include "../config/pin_config.h"
#include <fstream>
#include <chrono>
#define PID_LOGGING_ENABLED 1 // Use to enable or disable PID logging
#define KP_TEST 0 // Use to test different Kp values
double KP_GLOBAL = 0.01; // Proportional gain
std::chrono::time_point<std::chrono::system_clock> start_time;
// Declare a new publisher
rcl_publisher_t log_publisher;
rcl_publisher_t publisher;
rcl_subscription_t motor_subscriber;
rcl_subscription_t servo_subscriber;
std_msgs__msg__String motor_msg;
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

double log_kp;
double log_error;
double log_integral_error;
double log_derivative;
double log_new_pwm;
double log_current_speed;
double log_desired_speed;

void control_timer_callback(rcl_timer_t * control_timer, int64_t last_call_time)
{

    RCLC_UNUSED(last_call_time);
    if (control_timer != NULL) {

        std_msgs__msg__String__init(&control_msg);
        std::string data = std::to_string(servo.getAngle()) + " " + std::to_string(motor.getSpeed());
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
        log_data << /*timestamp << "," <<*/ log_kp << "," << log_error << "," << log_integral_error << "," << log_derivative << "," << log_new_pwm << "," << log_current_speed << "," << log_desired_speed;
        
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
	log_kp = 2;
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
	log_kp =1;
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    std::string msg_data = msg->data.data;
    
    // Find the position of the space character
    size_t space_pos = msg_data.find(' ');
    if (space_pos == std::string::npos) {
        // Invalid message format, handle error or return
        return;
    }

    // Extract pwmValue and direction
    double desired_speed = std::stod(msg_data.substr(0, space_pos));
    std::string direction = msg_data.substr(space_pos + 1);

    // Validate direction and pwm
    if (!isValidDirection(direction)){ //|| !isValidPwm(desired_pwm)) {
        // Invalid direction or pwm value, handle error or return
        return;
    }
    // Update the motor direction
    // check to see if direction has changed before updating
    if (motor.getDirection() != direction) {
        bool forward = (direction == "forward");
        bool reverse = (direction == "reverse");
        motor.updateDirection(reverse, forward, direction);
    }
    double Kp = 4.0;
    // Use Ziegler–Nichols method to tune the PID controller
    /*1. Find Kmax the value of Kp where it begins to oscillate
      2. Measure the Tu period of the Kmax oscillation
      3. Set Kp = 0.6*Kmax, Ki = 2*Kp/Tu, Kd = Kp*(Tu/8)
    */
 
    double Ki = 0.0; // Integral gain, tweak this value
    double Kd = 0.13; // Derivative gain, tweak this value
    // Measure the current speed
    double current_speed = motor.getSpeed();

    // Calculate the error
    double error = desired_speed - current_speed;

    // Calculate the integral term
    motor.integral_error += error;

    // Calculate the derivative term
    double derivative = error - motor.previous_error;
   

    // Adjust the PWM based on the error
    double new_pwm = motor.getCurrentPwm() + Kp * error + Ki * motor.integral_error + Kd * derivative;

    // Set the new PWM value to the motor
    motor.setSpeed(new_pwm);

    // Update the previous error
    motor.previous_error = error;

	if(PID_LOGGING_ENABLED){
		log_kp = 1;
		log_error = error;
		log_integral_error = motor.integral_error;
		log_derivative = derivative;
		log_new_pwm = new_pwm;
		log_desired_speed = desired_speed;
		log_current_speed = current_speed;
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
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
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
