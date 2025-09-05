#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mutex>

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node {
public:
    BatteryNode()
        : Node("battery_node")
        , battery_percentage(0.0f)
        , connected(false) {

        // Subscriber Initialization
        this->state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&BatteryNode::state_callback, this, std::placeholders::_1));
        this->battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", rclcpp::SensorDataQoS(), std::bind(&BatteryNode::battery_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->battery_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/battery", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(1s, std::bind(&BatteryNode::data_publisher, this));

        RCLCPP_INFO(this->get_logger(), "Battery node started");
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->state_mutex);
        this->connected = msg->connected;
    }

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->battery_mutex);
        this->battery_percentage = msg->percentage * 100.0f;
    }

    void data_publisher() {
        bool current_connected;
        float current_battery;

        {
            std::lock_guard<std::mutex> lock1(this->state_mutex);
            current_connected = this->connected;
        }

        {
            std::lock_guard<std::mutex> lock2(this->battery_mutex);
            current_battery = this->battery_percentage;
        }

        if (current_connected) {
            auto message = std_msgs::msg::Float32();
            message.data = current_battery;
            this->battery_pub->publish(message);
            RCLCPP_INFO(this->get_logger(), "Battery: %.2f %%", current_battery);
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Mutex Declaration
    std::mutex state_mutex;
    std::mutex battery_mutex;

    // Variable Declaration
    float battery_percentage;
    bool connected;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}