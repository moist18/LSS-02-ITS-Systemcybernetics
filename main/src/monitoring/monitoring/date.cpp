#include <chrono>
#include <ctime>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

class DateNode : public rclcpp::Node {
public:
    DateNode()
        : Node("date_node") {
        // Publishers
        date_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/date", 10);
        date_string_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/date_string", 10);

        // Timer for date updates
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DateNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Date node started");
    }

private:
    void timer_callback() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);

        // Get milliseconds
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // Create date array: [year, month, day, hour, minute, second, millisecond, day_of_week]
        auto date_msg = std_msgs::msg::Float32MultiArray();
        date_msg.data = {
            static_cast<float>(tm.tm_year + 1900), // Year
            static_cast<float>(tm.tm_mon + 1), // Month (1-12)
            static_cast<float>(tm.tm_mday), // Day of month
            static_cast<float>(tm.tm_hour), // Hour
            static_cast<float>(tm.tm_min), // Minute
            static_cast<float>(tm.tm_sec), // Second
            static_cast<float>(ms.count()), // Millisecond
            static_cast<float>(tm.tm_wday) // Day of week (0=Sunday)
        };
        date_pub_->publish(date_msg);

        // Create human-readable date string
        std::stringstream ss;
        ss << std::put_time(&tm, "%A, %d %B %Y %H:%M:%S");

        auto date_string_msg = std_msgs::msg::String();
        date_string_msg.data = ss.str();
        date_string_pub_->publish(date_string_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Date: %s", date_string_msg.data.c_str());
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr date_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr date_string_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}