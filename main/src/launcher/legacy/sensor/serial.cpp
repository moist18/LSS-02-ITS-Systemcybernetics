#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>

class SerialReader : public rclcpp::Node {
public:
    SerialReader()
        : Node("serial_node") {

        // Serial Configuration
        this->serial_port.setPort("/dev/ttyACM0");
        this->serial_port.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        this->serial_port.setTimeout(timeout);

        try {
            this->serial_port.open();
            if (this->serial_port.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
        }

        // Publisher Initialization
        this->publisher = this->create_publisher<std_msgs::msg::String>("/serial_data", 10);

        // Rate Timer Initialization
        this->timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialReader::read_serial, this));
    }

private:
    void read_serial() {
        if (this->serial_port.isOpen() && this->serial_port.available()) {
            std_msgs::msg::String msg;
            msg.data = this->serial_port.readline();
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg.data.c_str());
            this->publisher->publish(msg);
        }
    }

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr timer;

    // Serial Declaration
    serial::Serial serial_port;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReader>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
