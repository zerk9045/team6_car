
// Include the C++ standard library headers
#include <chrono>
#include <functional>
#include <string>
#include <memory> // Dynamic memory management
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
#include "geometry_msgs/msg/twist_stamped.hpp"


class VelocitySubscriberNode : public rclcpp::Node
{
public:
  VelocitySubscriberNode()
  : Node("velocity_subscriber_node")
  {
    // Subscribe to the /cmd_vel_stamped topic
    cmd_vel_stamped_subscriber_ =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_stamped",
      rclcpp::SensorDataQoS(),
      std::bind(&VelocitySubscriberNode::cmd_vel_stamped_callback, this, std::placeholders::_1));
  }

private:


  // Cmd Vel Stamped subscription callback
  void cmd_vel_stamped_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {

    // Echo the linear velocity components in the terminal where the node was started
    RCLCPP_INFO(this->get_logger(), "Linear Velocity: '%f'", msg->twist.linear.x);

    // Echo the angular velocity components in the terminal where the node was started
    RCLCPP_INFO(this->get_logger(), "Angular Velocity: '%f'", msg->twist.angular.z); 

  }

  // Cmd Vel StampedSubscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_subscriber_;
};



int main(int argc, char * argv[])
{
  // Launch ROS 2  
  rclcpp::init(argc, argv);

  // Prepare to receive messages that arrive on the topic  
  rclcpp::spin(std::make_shared<VelocitySubscriberNode>());

  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}
