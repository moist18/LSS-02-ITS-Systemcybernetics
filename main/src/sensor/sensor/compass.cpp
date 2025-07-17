#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class CompassNode : public rclcpp::Node {
public:
    CompassNode()
        : Node("compass_node") {
        // Subscribers
        compass_hdg_sub_ = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", 10,
            std::bind(&CompassNode::compass_callback, this, std::placeholders::_1));

        compass_control_sub_ = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", 10,
            std::bind(&CompassNode::compass_control_callback, this, std::placeholders::_1));

        // Publishers
        compass_success_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/compass/success", 10);

        // Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&CompassNode::run, this));

        // Variables
        current_heading_ = 0.0;
        compass_control_ = false;
        compass_success_ = false;
        target_heading_ = 0.0;
    }

private:
    void compass_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_heading_ = fmod(msg->data, 360.0);
        if (current_heading_ < 0)
            current_heading_ += 360.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current Heading: %.1f°", current_heading_);
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        compass_control_ = msg->data;
        if (compass_control_) {
            // Set a target heading when compass control is activated
            target_heading_ = current_heading_;
            compass_success_ = false;
            RCLCPP_INFO(this->get_logger(), "Compass control activated, target heading: %.1f°", target_heading_);
        } else {
            compass_success_ = false;
        }
    }

    void run() {
        if (compass_control_) {
            // Simple compass logic - check if we're close to target heading
            double heading_error = fabs(current_heading_ - target_heading_);
            if (heading_error > 180.0) {
                heading_error = 360.0 - heading_error;
            }

            if (heading_error < 5.0) { // Within 5 degrees tolerance
                compass_success_ = true;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Compass control active - Current: %.1f°, Target: %.1f°, Error: %.1f°, Success: %s", current_heading_,
                target_heading_, heading_error, compass_success_ ? "true" : "false");
        }

        // Publish compass success status
        auto success_msg = std_msgs::msg::Bool();
        success_msg.data = compass_success_;
        compass_success_pub_->publish(success_msg);
    }

    // Variables
    double current_heading_;
    bool compass_control_;
    bool compass_success_;
    double target_heading_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_hdg_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compass_success_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompassNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}