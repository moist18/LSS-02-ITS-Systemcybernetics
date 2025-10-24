#include "rclcpp/qos.hpp"
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
        , prev_idle(0)
        , prev_total(0) {

        // Publisher Initialization
        this->cpu_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/cpu", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(1s, std::bind(&CPUNode::data_publisher, this));

        RCLCPP_INFO(this->get_logger(), "MONITORING → CPU node started");
    }

private:
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

        if (this->prev_total == 0) {
            this->prev_total = total;
            this->prev_idle = idle_total;
            return 0.0f;
        }

        unsigned long long delta_total = total - this->prev_total;
        unsigned long long delta_idle = idle_total - this->prev_idle;

        float usage = 0.0f;
        if (delta_total > 0) {
            usage = (static_cast<float>(delta_total - delta_idle) / delta_total) * 100.0f;
        }

        this->prev_total = total;
        this->prev_idle = idle_total;

        return usage;
    }

    void data_publisher() {
        float cpu_usage = get_cpu_usage();
        auto message = std_msgs::msg::Float32();
        message.data = cpu_usage;
        this->cpu_pub->publish(message);

        if (cpu_usage > 80.0f) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "MONITORING → CPU Usage is high: %.2f %%", cpu_usage);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "MONITORING → CPU Usage: %.2f %%", cpu_usage);
        }
    }

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Variable Declaration
    unsigned long long prev_idle;
    unsigned long long prev_total;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CPUNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}