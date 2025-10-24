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

        // Callback Group Initialization
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        // Subscribers Initialization
        this->arming_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/arming", rclcpp::ServicesQoS(), std::bind(&StateNode::arming_callback, this, std::placeholders::_1), sub_options);
        this->setmode_sub = this->create_subscription<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS(), std::bind(&StateNode::setmode_callback, this, std::placeholders::_1), sub_options);

        // Service client Initialization
        this->arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        this->mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

        RCLCPP_INFO(this->get_logger(), "STATE → node started");
    }

private:
    void arming_callback(const std_msgs::msg::Bool::SharedPtr msg) {
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
        RCLCPP_INFO(this->get_logger(), "STATE → Attempting to send ARMING command...");
        if (!this->arming_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "STATE → Arming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        auto response_callback = [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "STATE → Armed");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "STATE → Arming failed");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "STATE → Arming error: %s", e.what());
            }
        };

        this->arming_client->async_send_request(request, response_callback);
    }

    void disarm() {
        RCLCPP_INFO(this->get_logger(), "STATE → Attempting to send DISARMING command...");
        if (!this->arming_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "STATE → Disarming service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;

        auto response_callback = [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "STATE → Disarmed");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "STATE → Disarming failed");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "STATE → Disarming error: %s", e.what());
            }
        };

        this->arming_client->async_send_request(request, response_callback);
    }

    void set_mode(const std::string& mode) {
        RCLCPP_INFO(this->get_logger(), "STATE → Attempting to set %s mode...", mode.c_str());
        if (!this->mode_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "STATE → Mode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        for (int i = 1; i <= 3; ++i) {
            auto result = this->mode_client->async_send_request(request);
            try {
                auto response = result.get();
                if (response->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "STATE → Mode set: %s", mode.c_str());
                    return;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "STATE → Mode set failed [%d/3]: %s", i, mode.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "STATE → Mode error [%d/3]: %s", i, e.what());
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        RCLCPP_ERROR(this->get_logger(), "STATE → Mode set failed after 3 attempts: %s", mode.c_str());
    }

    // Callback Group Declaration
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arming_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr setmode_sub;

    // Service client Declaration
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client;
};

bool wait_for_mavros(rclcpp::Node::SharedPtr node, int timeout_seconds) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "STATE → Waiting for MAVROS...");

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
            RCLCPP_ERROR(node->get_logger(), "STATE → MAVROS timeout after %ds", timeout_seconds);
            return false;
        }

        if (elapsed > 0 && elapsed % 5 == 0) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "STATE → Waiting for MAVROS (%lds)", elapsed);
        }

        rclcpp::sleep_for(500ms);
    }

    if (!heartbeat_received) {
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "STATE → MAVROS connected");

    // Step 2: Verify service availability with a test call
    auto test_client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    if (!test_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "STATE → MAVROS services not available");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "STATE → MAVROS ready");
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto temp_node = std::make_shared<rclcpp::Node>("state_init");

    temp_node->declare_parameter<int>("mavros_init_timeout", 20);
    int timeout = temp_node->get_parameter("mavros_init_timeout").as_int();

    if (!wait_for_mavros(temp_node, timeout)) {
        RCLCPP_FATAL(temp_node->get_logger(), "STATE → MAVROS init failed");
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