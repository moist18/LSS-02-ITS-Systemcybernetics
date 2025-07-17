#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

class EmergencyNode : public rclcpp::Node {
public:
    EmergencyNode()
        : Node("emergency_node") {
        // Publishers
        emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/emergency", 10);

        // Parameters
        this->declare_parameter("emergency_pin", 13);
        this->declare_parameter("check_interval_ms", 500);

        emergency_pin_ = this->get_parameter("emergency_pin").as_int();
        int check_interval = this->get_parameter("check_interval_ms").as_int();

        // Initialize GPIO
        if (!setup_gpio()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup GPIO");
            return;
        }

        // Timer for checking emergency button
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(check_interval), std::bind(&EmergencyNode::check_emergency, this));

        RCLCPP_INFO(this->get_logger(), "Emergency monitoring node started on pin %d", emergency_pin_);
    }

    ~EmergencyNode() { cleanup_gpio(); }

private:
    bool setup_gpio() {
        try {
            // Export GPIO pin
            std::ofstream export_file("/sys/class/gpio/export");
            if (export_file.is_open()) {
                export_file << emergency_pin_;
                export_file.close();
            }

            // Wait a bit for the export to take effect
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Set direction to input
            std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(emergency_pin_) + "/direction";
            std::ofstream direction_file(direction_path);
            if (direction_file.is_open()) {
                direction_file << "in";
                direction_file.close();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set GPIO direction");
                return false;
            }

            // Set edge to both (rising and falling)
            std::string edge_path = "/sys/class/gpio/gpio" + std::to_string(emergency_pin_) + "/edge";
            std::ofstream edge_file(edge_path);
            if (edge_file.is_open()) {
                edge_file << "both";
                edge_file.close();
            }

            value_path_ = "/sys/class/gpio/gpio" + std::to_string(emergency_pin_) + "/value";

            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GPIO setup exception: %s", e.what());
            return false;
        }
    }

    void cleanup_gpio() {
        try {
            std::ofstream unexport_file("/sys/class/gpio/unexport");
            if (unexport_file.is_open()) {
                unexport_file << emergency_pin_;
                unexport_file.close();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "GPIO cleanup exception: %s", e.what());
        }
    }

    bool read_gpio_value() {
        try {
            std::ifstream value_file(value_path_);
            if (value_file.is_open()) {
                std::string value_str;
                std::getline(value_file, value_str);
                value_file.close();
                return (value_str == "0"); // LOW = true (emergency pressed for pull-up configuration)
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read GPIO value: %s", e.what());
        }
        return false;
    }

    void check_emergency() {
        bool emergency_state = read_gpio_value();

        // Publish emergency state
        auto msg = std_msgs::msg::Bool();
        msg.data = emergency_state;
        emergency_pub_->publish(msg);

        // Log state changes
        if (emergency_state != prev_emergency_state_) {
            if (emergency_state) {
                RCLCPP_WARN(this->get_logger(), "EMERGENCY BUTTON PRESSED!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Emergency button released");
            }
            prev_emergency_state_ = emergency_state;
        }

        // Throttled logging for status
        if (emergency_state) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Emergency button is PRESSED - System in emergency state");
        } else {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 10000, "Emergency monitoring active - Button not pressed");
        }
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    int emergency_pin_;
    std::string value_path_;
    bool prev_emergency_state_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}