#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class StateNode : public rclcpp::Node {
public:
    StateNode()
        : Node("state_node") {
        // Subscribers
        arming_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/arming", 10, std::bind(&StateNode::armingCallback, this, std::placeholders::_1));

        setmode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/setmode", 10, std::bind(&StateNode::setModeCallback, this, std::placeholders::_1));

        // Service clients
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arming_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr setmode_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;

    void armingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            arm();
        } else {
            disarm();
        }
    }

    void setModeCallback(const std_msgs::msg::String::SharedPtr msg) { setMode(msg->data); }

    void arm() {
        RCLCPP_INFO(this->get_logger(), "Attempting to send ARMING command...");
        if (!arming_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto result = arming_client_->async_send_request(request);
        try {
            auto response = result.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent ARMING command");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send ARMING command");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void disarm() {
        RCLCPP_INFO(this->get_logger(), "Attempting to send DISARMING command...");
        if (!arming_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto result = arming_client_->async_send_request(request);
        try {
            auto response = result.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent DISARMING command");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send DISARMING command");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void setMode(const std::string& mode) {
        RCLCPP_INFO(this->get_logger(), "Attempting to set %s mode...", mode.c_str());
        if (!mode_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Mode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        for (int i = 1; i <= 3; ++i) {
            auto result = mode_client_->async_send_request(request);
            try {
                auto response = result.get();
                if (response->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "Successfully set to %s mode", mode.c_str());
                    return;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[%d] Failed to set %s mode", i, mode.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[%d] Service call failed for %s mode: %s", i, mode.c_str(), e.what());
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to set %s mode after 3 attempts", mode.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}