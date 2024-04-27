#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>
#include <Motor.h>
#include <string>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
Motor motor;
Servo servo;
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish your control command
    std_msgs__msg__String__init(&msg);
    msg.data.data = "Servo = " + std::to_string(servo.getAngle()) + "\nMotor = " + std::to_string(motor.getSpeed());
    msg.data.size = strlen(msg.data.data);
    msg.data.capacity = msg.data.size + 1;
    rcl_publish(&publisher, &msg, NULL);
    printf("Published: '%s'\n", msg.data.data);
  }
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("I heard: '%s'\n", msg->data.data);
    // Control motor and servo based on the received message

    // Parse the message to extract the control command
    std::string data(msg->data.data);
    std::size_t pos = data.find("=");

    if (pos != std::string::npos) {
      std::string pwmStr = data.substr(pos + 1); // Get the part after "="
      int pwm = std::stoi(pwmStr); // Convert to int
    }

    // Determine if the control command is for the motor or the servo based on string message
    if (msg->data.data[0] == 'S') {
      // Control the motor
      motor.setSpeed(pwm); // Set the speed of the motor
    } else {
    // Control the servo
     servo.setAngle(pwm); // Set the angle of the servo
  }
}

int main()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
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

  // create a subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "string_publisher");

  // create executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }

  // free resources
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rcl_node_fini(&node);
}