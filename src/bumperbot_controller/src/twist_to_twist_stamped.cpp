#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
    TwistToTwistStamped()
        : Node("twist_to_twist_stamped")
    {
        // Subscription to /key_vel (Twist messages)
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/key_vel", 10,
            std::bind(&TwistToTwistStamped::twistCallback, this, std::placeholders::_1));

        // Publisher to /input_joy/cmd_vel_stamped (TwistStamped messages)
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/input_joy/cmd_vel_stamped", 10);

        RCLCPP_INFO(this->get_logger(), "TwistToTwistStamped node has been started.");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Create a TwistStamped message
        auto twist_stamped = geometry_msgs::msg::TwistStamped();
        twist_stamped.header.stamp = this->get_clock()->now(); // Add the current timestamp
        twist_stamped.header.frame_id = "base_link";           // Optional: Set a frame ID
        twist_stamped.twist = *msg;                            // Copy the Twist message

        // Publish the TwistStamped message
        publisher_->publish(twist_stamped);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToTwistStamped>());
    rclcpp::shutdown();
    return 0;
}