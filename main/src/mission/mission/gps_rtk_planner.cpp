#include <chrono>
#include <map>
#include <mavros_msgs/msg/state.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode()
        : Node("planner_node") {

        // Parameter Declaration
        this->declare_parameter<double>("phase_duration.reverse", 3.0);

        // Parameter Setup
        load_parameter();

        // Subscriber Initialization
        this->state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&PlannerNode::state_callback, this, std::placeholders::_1));
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&PlannerNode::mode_callback, this, std::placeholders::_1));
        this->curr_wp_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/position/waypoint", rclcpp::ServicesQoS(), std::bind(&PlannerNode::curr_wp_callback, this, std::placeholders::_1));
        this->left_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS(), std::bind(&PlannerNode::left_buoy_callback, this, std::placeholders::_1));
        this->right_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS(), std::bind(&PlannerNode::right_buoy_callback, this, std::placeholders::_1));
        this->surface_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/surface", rclcpp::ServicesQoS(), std::bind(&PlannerNode::surface_callback, this, std::placeholders::_1));
        this->underwater_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS(), std::bind(&PlannerNode::underwater_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&PlannerNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->setmode_pub = this->create_publisher<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS());
        this->gps_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS());
        this->vision_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS());
        this->mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS());
        this->left_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS());
        this->right_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(20ms, std::bind(&PlannerNode::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&PlannerNode::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "PLANNER → node started");
    }

private:
    void load_parameter() {
        this->phase_duration.clear();
        this->phase_duration["reverse"] = this->get_parameter("phase_duration.reverse").as_double();
    }

    void param_callback() {
        this->load_parameter();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLANNER → Parameters Reloaded!");
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_state = *msg;
    }

    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->auto_mode = msg->data;
    }

    void curr_wp_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_waypoint = msg->data;
        this->nav_start_time = rclcpp::Time();
        this->reverse_started = false;
    }

    void left_buoy_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->default_left = msg->data;
        this->left_buoy = msg->data;
    }

    void right_buoy_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->default_right = msg->data;
        this->right_buoy = msg->data;
    }

    void surface_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->is_surface = msg->data;
    }

    void underwater_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->is_underwater = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->current_waypoint = 0;
            this->nav_start_time = rclcpp::Time();
            this->reverse_started = false;
            this->mission = "";
            this->left_buoy = this->default_left;
            this->right_buoy = this->default_right;
            RCLCPP_INFO(this->get_logger(), "PLANNER → Successfully Reset Mission!");
        }
    }

    double get_phase_duration(const std::string& phase_id) {
        if (this->phase_duration.find(phase_id) != this->phase_duration.end()) {
            return this->phase_duration[phase_id];
        }
        return 0.0;
    }

    void set_navigation_control(const std::string& mode, bool gps, bool compass, bool vision) {
        if (this->current_state.mode != mode) {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = mode;
            this->setmode_pub->publish(mode_msg);
        }

        this->gps_control = gps;
        this->compass_control = compass;
        this->vision_control = vision;

        auto gps_msg = std_msgs::msg::Bool();
        gps_msg.data = gps;
        this->gps_control_pub->publish(gps_msg);

        auto compass_msg = std_msgs::msg::Bool();
        compass_msg.data = compass;
        this->compass_control_pub->publish(compass_msg);

        auto vision_msg = std_msgs::msg::Bool();
        vision_msg.data = vision;
        this->vision_control_pub->publish(vision_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLANNER → Mode:%s | GPS:%d Compass:%d Vision:%d", mode.c_str(), gps, compass, vision);
    }

    void gps_navigation() {
        this->set_navigation_control("GUIDED", true, false, false);
    }

    void compass_navigation() {
        this->set_navigation_control("MANUAL", false, true, false);
    }

    void vision_navigation() {
        this->set_navigation_control("MANUAL", false, false, true);
    }

    void handle_waypoint_0(double current_time) {
        // Waypoint 0: Floating Ball mission - GPS navigation only
        this->mission = "Floating Ball";
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 0: Floating Ball - GPS");
    }

    void handle_waypoint_1(double current_time) {
        // Waypoint 1: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 1: GPS");
    }

    void handle_waypoint_2(double current_time) {
        // Waypoint 2: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: GPS");
    }

    void handle_waypoint_3(double current_time) {
        // Waypoint 3: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 3: GPS");
    }

    void handle_waypoint_4(double current_time) {
        // Waypoint 4: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 4: GPS");
    }

    void handle_waypoint_5(double current_time) {
        // Waypoint 5: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 5: GPS");
    }

    void handle_waypoint_6(double current_time) {
        // Waypoint 6: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 6: GPS");
    }

    void handle_waypoint_7(double current_time) {
        // Waypoint 7: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 7: GPS");
    }

    void handle_waypoint_8(double current_time) {
        // Waypoint 8: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 8: GPS");
    }

    void handle_waypoint_9(double current_time) {
        // Waypoint 9: GPS navigation
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 9: GPS");
    }

    void handle_waypoint_10(double current_time) {
        // Waypoint 10: Surface Imaging - Vision until surface complete, then reverse timer, then GPS
        this->mission = "Surface Imaging";

        if (!this->is_surface) {
            this->vision_navigation();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 10: Surface Imaging - Vision");
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            try {
                double phase_duration = this->get_phase_duration("reverse");
                double elapsed_time = current_time - this->nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    this->gps_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 10: Surface Imaging - GPS");
                } else {
                    this->vision_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 10: Surface Imaging - Vision");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 10: %s", e.what());
            }
        }
    }

    void handle_waypoint_11(double current_time) {
        // Waypoint 11: Underwater Imaging - GPS navigation
        this->mission = "Underwater Imaging";
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 11: Underwater Imaging - GPS");
    }

    void handle_waypoint_12(double current_time) {
        // Waypoint 12: Vision until underwater complete, then reverse timer, then GPS
        if (!this->is_underwater) {
            this->vision_navigation();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 12: Underwater Imaging - Vision");
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            try {
                double phase_duration = this->get_phase_duration("reverse");
                double elapsed_time = current_time - this->nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    this->gps_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 12: Underwater Imaging - GPS");
                } else {
                    this->vision_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 12: Underwater Imaging - Vision");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 12: %s", e.what());
            }
        }
    }

    void handle_waypoint_13(double current_time) {
        // Waypoint 13: Docking - GPS navigation
        this->mission = "Docking";
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 13: Docking - GPS");
    }

    void handle_waypoint_14(double current_time) {
        // Waypoint 14: Vision navigation - road to finish
        this->vision_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 14: Road to Finish!");
    }

    void mission_planner() {
        std::lock_guard<std::mutex> lock(this->mutex);
        auto now = this->now();

        if (!this->auto_mode) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLANNER → RC AUTO mode is disabled, waiting for RC AUTO mode...");
            return;
        }

        // Get current time in seconds
        double current_time = now.seconds();

        switch (this->current_waypoint) {
        case 0:
            this->handle_waypoint_0(current_time);
            break;
        case 1:
            this->handle_waypoint_1(current_time);
            break;
        case 2:
            this->handle_waypoint_2(current_time);
            break;
        case 3:
            this->handle_waypoint_3(current_time);
            break;
        case 4:
            this->handle_waypoint_4(current_time);
            break;
        case 5:
            this->handle_waypoint_5(current_time);
            break;
        case 6:
            this->handle_waypoint_6(current_time);
            break;
        case 7:
            this->handle_waypoint_7(current_time);
            break;
        case 8:
            this->handle_waypoint_8(current_time);
            break;
        case 9:
            this->handle_waypoint_9(current_time);
            break;
        case 10:
            this->handle_waypoint_10(current_time);
            break;
        case 11:
            this->handle_waypoint_11(current_time);
            break;
        case 12:
            this->handle_waypoint_12(current_time);
            break;
        case 13:
            this->handle_waypoint_13(current_time);
            break;
        case 14:
            this->handle_waypoint_14(current_time);
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "PLANNER → Invalid waypoint: %d", this->current_waypoint);
        }
    }

    bool check_state() {
        std::lock_guard<std::mutex> lock(this->mutex);

        if (!this->current_state.connected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FCU is not connected, waiting for FCU connection...");
            return false;
        }

        if (!this->current_state.armed) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot is DISARMED, waiting for ARMING command...");
            return false;
        }

        if (this->current_state.mode != "GUIDED" && this->current_state.mode != "MANUAL") {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot is in %s mode, waiting for GUIDED or MANUAL mode...", this->current_state.mode.c_str());
            return false;
        }

        return true;
    }

    void runner() {
        if (!this->check_state()) {
            return;
        }

        this->mission_planner();

        {
            std::lock_guard<std::mutex> lock(this->mutex);

            auto mission_msg = std_msgs::msg::String();
            mission_msg.data = this->mission;
            this->mission_pub->publish(mission_msg);

            auto left_msg = std_msgs::msg::String();
            left_msg.data = this->left_buoy;
            this->left_buoy_pub->publish(left_msg);

            auto right_msg = std_msgs::msg::String();
            right_msg.data = this->right_buoy;
            this->right_buoy_pub->publish(right_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Current mission: %s", this->mission.c_str());
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr curr_wp_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_buoy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_buoy_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr surface_mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr underwater_mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setmode_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gps_control_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compass_control_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_control_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_buoy_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_buoy_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Variable Declaration
    std::mutex mutex;
    rclcpp::Time nav_start_time;
    mavros_msgs::msg::State current_state;
    std::map<std::string, double> phase_duration;
    std::string mission;

    // Variable Initialization
    std::string default_left = "green_buoy";
    std::string default_right = "red_buoy";
    std::string left_buoy = "green_buoy";
    std::string right_buoy = "red_buoy";
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool is_surface = false;
    bool is_underwater = false;
    bool reverse_started = false;
    int current_waypoint = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}