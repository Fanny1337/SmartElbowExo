#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>
#include <array>

class DynamicModelNode : public rclcpp::Node
{
public:
    DynamicModelNode() : Node("dynamic_model")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "elbow_torque", 10,
            std::bind(&DynamicModelNode::torque_callback, this, std::placeholders::_1));

        position_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "elbow_position_cmd", 10);      

        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "elbow_velocity", 10);

        current_state_ = {0.0, 0.0}; // Initial position and velocity

        // Create a timer to call publish_state every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DynamicModelNode::publish_state, this));
    }

private:
    void torque_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double dt = 1.0; // Simulation timestep in seconds
        auto new_state = rk4_integration(current_state_, msg->data, dt);
        current_state_ = new_state;
    }

    std::array<double, 2> rk4_integration(const std::array<double, 2>& state, double torque, double dt)
    {
        auto f = [this, torque](const std::array<double, 2>& y) -> std::array<double, 2> {
            double q1 = y[0];  // position
            double qp1 = y[1]; // velocity

            double accel = compute_acceleration(q1, qp1, torque);
            return {qp1, accel}; // {velocity, acceleration}
        };

        auto k1 = f(state);
        auto k2 = f({state[0] + 0.5 * dt * k1[0], state[1] + 0.5 * dt * k1[1]});
        auto k3 = f({state[0] + 0.5 * dt * k2[0], state[1] + 0.5 * dt * k2[1]});
        auto k4 = f({state[0] + dt * k3[0], state[1] + dt * k3[1]});

        double new_q1 = state[0] + (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
        double new_qp1 = state[1] + (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);

        return {new_q1, new_qp1};
    }

    double compute_acceleration(double q1, double qp1, double torque)
    {
        // Parameters for the arm
        double m1 = 1.6, cm1 = 0.16, Inertia1 = 0.013;

        // Parameters for the orthosis
        double m2 = 0.015, cm2 = 0.1, Inertia2 = 0.00004625;

        double g_0 = 9.81, Beta = 0.5; //borde denna vara 0.5 eller 0.1?

        // Inertia matrix
        Eigen::MatrixXd M(1,1);
        M(0,0) = (Inertia1 + cm1 * cm1 * m1) + (Inertia2 + cm2 * cm2 * m2);

        // Gravitational torques
        Eigen::MatrixXd G(1,1);
        G(0,0) = (cm1 * m1 * g_0 * cos(q1)) + (cm2 * m2 * g_0 * cos(q1));

        // Viscous friction
        Eigen::MatrixXd B(1,1);
        B(0,0) = Beta * qp1;

        // Compute acceleration
        Eigen::MatrixXd Qpp = M.inverse() * (Eigen::MatrixXd::Constant(1,1,torque) - G - B);
        return Qpp(0,0);
    }

    void publish_state()
    {
        std_msgs::msg::Float64 position_msg;
        position_msg.data = current_state_[0];
        position_publisher_->publish(position_msg);

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = current_state_[1];
        velocity_publisher_->publish(velocity_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<double, 2> current_state_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicModelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
