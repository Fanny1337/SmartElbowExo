#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>  // For trigonometric functions
#include <chrono> // For time-related operations

class PDForceControllerNode : public rclcpp::Node
{
public:
    PDForceControllerNode() : 
        Node("control_node"), 
        kp_(3), 
        kd_(1.0), 
        filtered_position_(0.0), 
        filtered_derivative_(0.0),
        last_filtered_position_(0.0),
        last_time_(std::chrono::steady_clock::now()),
        alpha_position_(0.1),  // Position filtering constant
        alpha_derivative_(0.05) // Derivative filtering constant
    {
        desired_position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "elbow_position_des", 10,
            std::bind(&PDForceControllerNode::desiredPositionCallback, this, std::placeholders::_1));

        //actual_position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
          //  "elbow_position_cmd", 10,
          //  std::bind(&PDForceControllerNode::actualPositionCallback, this, std::placeholders::_1));

        torque_publisher_ = this->create_publisher<std_msgs::msg::Float64>("elbow_torque", 10);
    }

private:
    void desiredPositionCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        filtered_position_ = alpha_position_ * msg->data + (1 - alpha_position_) * filtered_position_;
        calculateAndPublishTorque();
    }

    void actualPositionCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time_).count();
        last_time_ = now;

        double raw_position = msg->data;
        double filtered_actual_position = alpha_position_ * raw_position + (1 - alpha_position_) * last_filtered_position_;
        last_filtered_position_ = filtered_actual_position;

        double derivative = (filtered_actual_position - last_filtered_position_) / dt;
        filtered_derivative_ = alpha_derivative_ * derivative + (1 - alpha_derivative_) * filtered_derivative_;

        calculateAndPublishTorque();
    }

    void calculateAndPublishTorque()
    {
        double position_error = filtered_position_ - last_filtered_position_;
        double control_torque = kp_ * position_error + kd_ * filtered_derivative_;
        control_torque = clamp_torque(control_torque, -10.0, 10.0);  // Manually limit torque to ±10 Nm

        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = control_torque;
        torque_publisher_->publish(torque_msg);
    }

    double clamp_torque(double value, double min, double max)
    {
        return std::max(min, std::min(value, max));
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr desired_position_subscription_;
   // rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr actual_position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_publisher_;

    double kp_, kd_;
    double filtered_position_, filtered_derivative_, last_filtered_position_;
    std::chrono::steady_clock::time_point last_time_;
    double alpha_position_, alpha_derivative_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PDForceControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
