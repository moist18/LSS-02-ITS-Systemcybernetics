#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class CompassNode : public rclcpp::Node {
public:
    CompassNode()
        : Node("compass_node") {

        // Subscriber Initialization
        this->compass_hdg_sub = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", 10, std::bind(&CompassNode::compass_callback, this, std::placeholders::_1));
        this->compass_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", 10, std::bind(&CompassNode::compass_control_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->compass_success_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/compass/success", 10);

        // Rate Timer Initialization
        this->timer = this->create_wall_timer(100ms, std::bind(&CompassNode::run, this));
    }

private:
    void compass_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        this->current_heading = fmod(msg->data, 360.0);
        if (this->current_heading < 0)
            this->current_heading += 360.0;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current Heading: %.1f°", this->current_heading);
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->compass_control = msg->data;
        if (this->compass_control) {
            // Set a target heading when compass control is activated
            this->target_heading = this->current_heading;
            this->compass_success = false;
            RCLCPP_INFO(this->get_logger(), "Compass control activated, target heading: %.1f°", this->target_heading);
        } else {
            this->compass_success = false;
        }
    }

    void run() {
        if (this->compass_control) {
            // Simple compass logic - check if we're close to target heading
            double heading_error = fabs(this->current_heading - this->target_heading);
            if (heading_error > 180.0) {
                heading_error = 360.0 - heading_error;
            }

            if (heading_error < 5.0) { // Within 5 degrees tolerance
                this->compass_success = true;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Compass control active - Current: %.1f°, Target: %.1f°, Error: %.1f°, Success: %s", this->current_heading, this->target_heading, heading_error,
                this->compass_success ? "true" : "false");
        }

        // Publish compass success status
        auto success_msg = std_msgs::msg::Bool();
        success_msg.data = this->compass_success;
        this->compass_success_pub->publish(success_msg);
    }

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_hdg_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compass_success_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr timer;

    // Variable Initialization
    double current_heading = 0.0;
    bool compass_control = false;
    bool compass_success = false;
    double target_heading = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompassNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}