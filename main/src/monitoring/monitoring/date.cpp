#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class DateNode : public rclcpp::Node {
public:
    DateNode()
        : Node("date_node") {
        // Publisher Initialization
        this->date_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/date", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(1s, std::bind(&DateNode::data_publisher, this));

        RCLCPP_INFO(this->get_logger(), "Date node started");
    }

private:
    void data_publisher() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto tm = *std::localtime(&time_t);

        // Get day name
        std::string day_names[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
        std::string day = day_names[tm.tm_wday];

        // Format date string
        std::stringstream date_ss;
        date_ss << std::setfill('0') << std::setw(2) << tm.tm_mday << " ";
        std::string month_names[] = { "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December" };
        date_ss << month_names[tm.tm_mon] << " " << (tm.tm_year + 1900);
        std::string date = date_ss.str();

        // Format time string
        std::stringstream time_ss;
        time_ss << std::setfill('0') << std::setw(2) << tm.tm_hour << ":" << std::setfill('0') << std::setw(2) << tm.tm_min << ":" << std::setfill('0') << std::setw(2) << tm.tm_sec;
        std::string time = time_ss.str();

        auto date_data = std_msgs::msg::Float32MultiArray();
        date_data.data = {
            static_cast<float>(tm.tm_wday), // Day of week (0=Sunday, 6=Saturday)
            static_cast<float>(tm.tm_mday), // Day of month
            static_cast<float>(tm.tm_mon + 1), // Month (1-12)
            static_cast<float>(tm.tm_year + 1900), // Year
            static_cast<float>(tm.tm_hour), // Hour
            static_cast<float>(tm.tm_min), // Minute
            static_cast<float>(tm.tm_sec) // Second
        };

        this->date_pub->publish(date_data);
        RCLCPP_INFO(this->get_logger(), "Datetime: %s, %s [%s]", day.c_str(), date.c_str(), time.c_str());
    }

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr date_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DateNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}