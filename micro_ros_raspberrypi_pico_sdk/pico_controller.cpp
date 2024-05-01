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
        std::string data = "Servo = " + std::to_string(servo.getAngle()); + "\nMotor = " + std::to_string(motor.getSpeed());
        msg.data.data = strdup(data.c_str()); // Create a copy of the string
        msg.data.size = strlen(msg.data.data);
        msg.data.capacity = msg.data.size + 1;
        rcl_publish(&publisher, &msg, NULL);
        printf("Published: '%s'\n", msg.data.data);
        free(msg.data.data); // Free the allocated memory
    }
}
void subscription_callback_servo(const void * msgin) {
    const std_msgs__msg__String * servo_msg = (const std_msgs__msg__String *)msgin;
    int pwm = std::stoi(servo_msg->data.data);
    servo.setAngle(pwm);

}

void subscription_callback_motor(const void *msgin) {
    const std_msgs__msg__String *motor_msg = (const std_msgs__msg__String *)msgin;
    std::string msg_data = motor_msg->data.data;

    // Find the position of the space character
    size_t space_pos = msg_data.find(' ');
    if (space_pos == std::string::npos) {
        // Invalid message format, handle error or return
        return;
    }

    // Extract pwmValue and direction
    int pwm = std::stoi(msg_data.substr(0, space_pos));
    std::string direction = msg_data.substr(space_pos + 1);

    // Set motor direction based on the extracted direction string
    bool forward = (direction == "forward");
    bool reverse = (direction == "reverse");
    motor.updateDirection(forward, reverse);

    // Set the motor speed
    motor.setSpeed(pwm);
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
  const unsigned int timer_timeout = 1000;
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
  rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &subscription_callback_motor, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &subscription_callback_servo, ON_NEW_DATA);
  //motor.setSpeed(1600000);

  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }

  // free resources
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&motor_subscriber, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_node_fini(&node);
}