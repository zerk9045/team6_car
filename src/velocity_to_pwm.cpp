
// Include the C++ standard library headers
#include <memory> // Dynamic memory management
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
 
class VelocitySubscriber : public rclcpp::Node
{
  public:
    // Constructor
    // The name of the node is minimal_subscriber
    VelocitySubscriber()
    : Node("velocity_subscriber")
    {
      // Create the subscription.
      // The topic_callback function executes whenever data is published
      // to the 'addison' topic.
      subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "velocity_subscriber", 10, std::bind(&VelocitySubscriber::topic_callback, this, _1));
    }
 
  private:
    // Receives the String message that is published over the topic
    void topic_callback(const geometry_msgs::msg::TwistStamped msg) const
    {
      // Write the message that was received on the console window
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    // Declare the subscription attribute
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<VelocitySubscriber>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}