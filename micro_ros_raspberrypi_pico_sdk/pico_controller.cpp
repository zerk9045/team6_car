#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>
#include <Motor.h>
#include <string>
#include <rmw_microros/rmw_microros.h>
#include <sstream>
#include <vector>
#include <iterator>
#include "pico_uart_transports.h"
#include <rcl/error_handling.h>

rcl_publisher_t publisher;
rcl_subscription_t motor_subscriber;
rcl_subscription_t servo_subscriber;
std_msgs__msg__String motor_msg;
std_msgs__msg__String msg;
std_msgs__msg__String servo_msg;
Motor motor;
Servo servo;
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        std_msgs__msg__String__init(&msg);
        std::string data = std::to_string(servo.getAngle()) + " " + std::to_string(motor.getSpeed());
        msg.data.data = strdup(data.c_str()); // Create a copy of the string
        msg.data.size = strlen(msg.data.data);
        msg.data.capacity = msg.data.size + 1;
        rcl_publish(&publisher, &msg, NULL);
        //printf("Published: '%s'\n", msg.data.data);
        free(msg.data.data); // Free the allocated memory
    }
}
//Test the motor
void testMotor(){
    motor.setSpeed(1);
    sleep_ms(1000);
    motor.setSpeed(0.0);
    sleep_ms(1000);
    motor.setSpeed(-1);
    sleep_ms(1000);
    motor.setSpeed(0.0);
}

//Function to test the PID controller
void testPIDController() {
    motor.getSpeed();
    // Set a desired speed
    double desiredSpeed = 1;
    motor.pidController(desiredSpeed);

    // Wait for the motor to reach the desired speed
    sleep_ms(500);

    // Get the actual speed of the motor
    double actualSpeed = motor.getSpeed();

    // Print the desired and actual speeds
    printf("Desired Speed: %f, Actual Speed: %f\n", desiredSpeed, actualSpeed);

    // Repeat the process for a different desired speed
    motor.getSpeed();
    desiredSpeed = -1;
    motor.pidController(desiredSpeed);
    sleep_ms(500);
    actualSpeed = motor.getSpeed();
    printf("Desired Speed: %f, Actual Speed: %f\n", desiredSpeed, actualSpeed);
}

// TODO: add error checks to ensure the message is in the correct format
void subscription_callback_servo(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    int pwm = std::stoi(msg->data.data);
    servo.setAngle(pwm);

}

void subscription_callback_motor(const void *msgin) {
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    double speed = std::stod(msg->data.data);
    // Set the motor speed
    motor.getSpeed();
    motor.pidController(speed);
}

int main()
{

    stdio_init_all();
    rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
    );
    rcl_allocator_t allocator = rcl_get_default_allocator();
    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

  rclc_support_t support;

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create rcl_node
  rcl_node_t node;
  rclc_node_init_default(&node, "pico_node", "", &support);

  // create a publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "control_topic");


  // create a timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 100;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "pi_servo_publishing_topic");

  // create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &motor_subscriber, &msg, &subscription_callback_motor, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_subscriber, &msg, &subscription_callback_servo, ON_NEW_DATA);
    //Tests
    //testMotor();
//    testPIDController();
//    motor.setSpeed(1.0);
  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }

  // free resources
  rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rcl_subscription_fini(&motor_subscriber, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_node_fini(&node);
}