#include <chrono>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

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

        RCLCPP_INFO(this->get_logger(), "✅ State node initialized");
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

bool wait_for_mavros(rclcpp::Node::SharedPtr node, int timeout_seconds) {
    RCLCPP_INFO(node->get_logger(), "⏳ Waiting for MAVROS to be ready...");

    // Step 1: Wait for /mavros/state topic to show connection
    bool heartbeat_received = false;
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), [&heartbeat_received](const mavros_msgs::msg::State::SharedPtr msg) {
        if (msg->connected) {
            heartbeat_received = true;
        }
    });

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !heartbeat_received) {
        rclcpp::spin_some(node);
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count();

        if (elapsed > timeout_seconds) {
            RCLCPP_ERROR(node->get_logger(), "❌ Timeout waiting for MAVROS heartbeat after %ds", timeout_seconds);
            return false;
        }

        if (elapsed > 0 && elapsed % 5 == 0) {
            RCLCPP_INFO(node->get_logger(), "   Still waiting for MAVROS connection... (%lds)", elapsed);
        }

        rclcpp::sleep_for(500ms);
    }

    if (!heartbeat_received) {
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "✅ MAVROS heartbeat detected");

    // Step 2: Verify service availability with a test call
    auto test_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    if (!test_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "❌ MAVROS arming service not available");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "✅ MAVROS services ready");
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create temporary node with different name to avoid conflict
    auto temp_node = std::make_shared<rclcpp::Node>("state_init");

    // Declare parameter with default value (will be overridden by launch file if provided)
    temp_node->declare_parameter<int>("mavros_init_timeout", 20);
    int timeout = temp_node->get_parameter("mavros_init_timeout").as_int();

    if (!wait_for_mavros(temp_node, timeout)) {
        RCLCPP_FATAL(temp_node->get_logger(), "❌ MAVROS initialization failed - State node cannot start");
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<StateNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}