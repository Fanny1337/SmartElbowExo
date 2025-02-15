#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "turtle_msgs/msg/exo_state.hpp"
#include <libserialport.h>
#include <cmath>  
#include <sstream>
#include <string>
#include <vector>

class SerialCommunicationNode : public rclcpp::Node
{
public:
    SerialCommunicationNode() : Node("serial_communication_node")
    {
        sp_get_port_by_name("/dev/ttyUSB0", &port);
        if (sp_open(port, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
            return;
        }
        sp_set_baudrate(port, 9600);

        position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "position_cmd",
            10,
            std::bind(&SerialCommunicationNode::position_command_callback, 
                this, 
                std::placeholders::_1));

        exo_state_publisher_ = this->create_publisher<turtle_msgs::msg::ExoState>("exo_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&SerialCommunicationNode::read_serial, this));
    }

    ~SerialCommunicationNode()
    {
        sp_close(port);
        sp_free_port(port);
    }

private:
    void position_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double radians = msg->data;
        int degrees = static_cast<int>(std::round(radians * 180.0 / M_PI)); // Convert radians to degrees
        degrees = std::clamp(degrees, 0, 90); // Ensure the degree is within the servo's range

        std::string command = std::to_string(degrees) + "\n";
        sp_nonblocking_write(port, command.c_str(), command.length());
    }

    void read_serial()
    {
        char buf[256];
        int bytes_read = sp_nonblocking_read(port, buf, sizeof(buf) - 1);
        if (bytes_read > 0) {
            buf[bytes_read] = '\0'; // Null-terminate the string
            std::string data(buf); 
            process_data(data); // Correctly call process_data using the data string
            
        } else if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port");
        }
    }

    void process_data(const std::string& data)
    {
        std::istringstream stream(data);
        std::string line;
        while (std::getline(stream, line)) {
            if (!line.empty()) {
                auto parts = split(line, ',');
                if (parts.size() == 3) {
                    try {
                        float position = std::stof(parts[0]); // Position in degrees
                        position = position * M_PI / 180.0; // Convert to radians for publishing

                        turtle_msgs::msg::ExoState msg;
                        msg.header.stamp = now();
                        msg.jdata = position;
                        msg.tdata.at(0) = std::stoi(parts[1]);
                        msg.tdata.at(1) = std::stoi(parts[2]);

                        exo_state_publisher_->publish(msg);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Error parsing data: %s", e.what());
                    }
                }
            }
        }
    }

    std::vector<std::string> split(const std::string& s, char delimiter)
    {
        std::vector<std::string> tokens;
        std::istringstream tokenStream(s);
        std::string token;
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    struct sp_port *port;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<turtle_msgs::msg::ExoState>::SharedPtr exo_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialCommunicationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
