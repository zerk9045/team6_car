#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"

constexpr double angular_vel = 15.0;  // example angular velocity
constexpr double linear_vel = 10.0;   // example linear velocity
constexpr double max_velocity = 15.0; // define maximum linear velocity
constexpr int min_pwm_duty_cycle = 1000000; // define minimum duty cycle
constexpr int max_pwm_duty_cycle = 2000000; // define maximum duty cycle

class VelocitySubscriber : public rclcpp::Node
{
public:
    VelocitySubscriber() : Node("velocity_subscriber")
    {
        subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel",
            10,
            this {
                RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
            });
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto velocity_subscriber = std::make_shared<VelocitySubscriber>();
    rclcpp::spin(velocity_subscriber);
    rclcpp::shutdown();
    return 0;
}
