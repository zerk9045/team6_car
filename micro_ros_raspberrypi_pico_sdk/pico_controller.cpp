#include <stdio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <Servo.h>
#include <Motor.h>

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish your control command
    std_msgs__msg__String__init(&msg);
    msg.data.data = "Your control command";
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
    //Control motor and servo based on the received message
    //Example: Motor motor; motor.setSpeed(50);

  }
}

int main()
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create rcl_node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

  // create a publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "control_topic"));

  // create a timer,
  rcl_timer_t timer;
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create a subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "string_publisher"));

  // create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  while(1){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }

  // free resources
  RCCHECK(rcl_publisher_fini(&publisher, &node));
  RCCHECK(rcl_subscription_fini(&subscriber, &node));
  RCCHECK(rcl_node_fini(&node));
}