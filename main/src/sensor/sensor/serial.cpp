#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>

class SerialReader : public rclcpp::Node {
public:
    SerialReader()
        : Node("serial_node") {
        // Konfigurasi port serial
        serial_port.setPort("/dev/ttyACM0");
        serial_port.setBaudrate(9600);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);

        try {
            serial_port.open();
            if (serial_port.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port!");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
        }

        // Publisher untuk topik /serial_data
        publisher_ = this->create_publisher<std_msgs::msg::String>("/serial_data", 10);

        // Timer untuk membaca data dari serial secara berkala
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialReader::read_serial, this));
    }

private:
    void read_serial() {
        if (serial_port.isOpen() && serial_port.available()) {
            std_msgs::msg::String msg;
            msg.data = serial_port.readline();
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg.data.c_str());
            publisher_->publish(msg);
        }
    }

    serial::Serial serial_port;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialReader>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
