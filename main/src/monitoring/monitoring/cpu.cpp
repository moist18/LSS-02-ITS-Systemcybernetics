#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class CPUNode : public rclcpp::Node {
public:
    CPUNode()
        : Node("cpu_node")
        , prev_idle_(0)
        , prev_total_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/robot/cpu", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&CPUNode::timer_callback, this));
    }

private:
    void timer_callback() {
        float cpu_usage = get_cpu_usage();
        auto message = std_msgs::msg::Float32();
        message.data = cpu_usage;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "CPU Usage: %.2f %%", cpu_usage);
    }

    float get_cpu_usage() {
        std::ifstream stat_file("/proc/stat");
        if (!stat_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /proc/stat");
            return 0.0f;
        }

        std::string line;
        std::getline(stat_file, line);
        stat_file.close();

        std::istringstream iss(line);
        std::string cpu_label;
        iss >> cpu_label;
        if (cpu_label != "cpu") {
            RCLCPP_ERROR(this->get_logger(), "Unexpected format in /proc/stat");
            return 0.0f;
        }

        unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
        iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

        unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;
        unsigned long long idle_total = idle + iowait;

        if (prev_total_ == 0) {
            prev_total_ = total;
            prev_idle_ = idle_total;
            return 0.0f;
        }

        unsigned long long delta_total = total - prev_total_;
        unsigned long long delta_idle = idle_total - prev_idle_;

        float usage = 0.0f;
        if (delta_total > 0) {
            usage = (static_cast<float>(delta_total - delta_idle) / delta_total) * 100.0f;
        }

        prev_total_ = total;
        prev_idle_ = idle_total;

        return usage;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    unsigned long long prev_idle_;
    unsigned long long prev_total_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CPUNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}