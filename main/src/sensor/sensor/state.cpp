#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class StateNode : public rclcpp::Node {
public:
    StateNode()
        : Node("state_node") {

        // Create callback group for subscriptions
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        // Subscribers Initialization
        this->arming_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/arming", rclcpp::ServicesQoS(), std::bind(&StateNode::arming_callback, this, std::placeholders::_1), sub_options);
        this->setmode_sub = this->create_subscription<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS(), std::bind(&StateNode::setmode_callback, this, std::placeholders::_1), sub_options);

        // Service client Initialization
        this->arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        this->mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        RCLCPP_INFO(this->get_logger(), "State node started");
    }

private:
    void arming_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "🔔 CALLBACK TRIGGERED! Received arming: %s", msg->data ? "ARM" : "DISARM");
        if (msg->data) {
            this->arm();
        } else {
            this->disarm();
        }
    }

    void setmode_callback(const std_msgs::msg::String::SharedPtr msg) {
        this->set_mode(msg->data);
    }

    void arm() {
        RCLCPP_INFO(this->get_logger(), "Attempting to send ARMING command...");
        if (!this->arming_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto response_callback = [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "✅ Successfully sent ARMING command");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ Failed to send ARMING command");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        };

        this->arming_client->async_send_request(request, response_callback);
    }

    void disarm() {
        RCLCPP_INFO(this->get_logger(), "Attempting to send DISARMING command...");
        if (!this->arming_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Arming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto response_callback = [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "✅ Successfully sent DISARMING command");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ Failed to send DISARMING command");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        };

        this->arming_client->async_send_request(request, response_callback);
    }

    void set_mode(const std::string& mode) {
        RCLCPP_INFO(this->get_logger(), "Attempting to set %s mode...", mode.c_str());
        if (!this->mode_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Mode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        for (int i = 1; i <= 3; ++i) {
            auto result = this->mode_client->async_send_request(request);
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

    // Callback group
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arming_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr setmode_sub;

    // Service client Declaration
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}