#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class TestControlNode : public rclcpp::Node
{
public:
    TestControlNode() : Node("test_control_node"), desired_position_(90.0), desired_velocity_(0.0)
    {
        // Publisher for the torque command
        torque_publisher_ = this->create_publisher<std_msgs::msg::Float64>("elbow_torque", 10);

        // Subscribers to the robot's current state
        position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "elbow_position", 10, std::bind(&TestControlNode::position_callback, this, std::placeholders::_1));
        velocity_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "elbow_velocity", 10, std::bind(&TestControlNode::velocity_callback, this, std::placeholders::_1));
    }

private:
    void position_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double position_error = desired_position_ - msg->data;
        update_control(position_error, last_velocity_error_);
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double velocity_error = desired_velocity_ - msg->data;
        last_velocity_error_ = velocity_error;  // Store last velocity error for control calculation
    }

    void update_control(double position_error, double velocity_error)
    {
        double kp = 1.0;  // Proportional gain
        double kd = 0.1;  // Derivative gain

        double torque = kp * position_error + kd * velocity_error;

        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = torque;
        torque_publisher_->publish(torque_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;

    double desired_position_;
    double desired_velocity_;
    double last_velocity_error_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
