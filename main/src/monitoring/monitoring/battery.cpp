#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node {
public:
    BatteryNode()
        : Node("battery_node")
        , battery_percentage_(0.0f)
        , connected_(false) {
        // Subscribers
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, [this](const mavros_msgs::msg::State::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                connected_ = msg->connected;
            });

        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(battery_mutex_);
                battery_percentage_ = msg->percentage * 100.0f;
            });

        // Publisher
        battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot/battery", 10);

        // Timer for publishing
        timer_ = this->create_wall_timer(1s, [this]() {
            bool current_connected;
            float current_battery;

            {
                std::lock_guard<std::mutex> lock1(state_mutex_);
                current_connected = connected_;
            }

            {
                std::lock_guard<std::mutex> lock2(battery_mutex_);
                current_battery = battery_percentage_;
            }

            if (current_connected) {
                auto message = std_msgs::msg::Float32();
                message.data = current_battery;
                battery_pub_->publish(message);
                RCLCPP_INFO(this->get_logger(), "Battery: %.2f %%", current_battery);
            }
        });
    }

private:
    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Shared data with mutex protection
    std::mutex state_mutex_;
    std::mutex battery_mutex_;
    float battery_percentage_;
    bool connected_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}