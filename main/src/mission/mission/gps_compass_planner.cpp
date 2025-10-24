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

class GPSCompassPlannerNode : public rclcpp::Node {
public:
    GPSCompassPlannerNode()
        : Node("gps_compass_planner_node")
        , nav_start_time(rclcpp::Time()) {

        // Parameter Declaration
        this->declare_parameter<double>("phase_duration.0", 10.0);
        this->declare_parameter<double>("phase_duration.1", 8.0);
        this->declare_parameter<double>("phase_duration.2", 8.0);
        this->declare_parameter<double>("phase_duration.reverse", 3.0);

        // Parameter Setup
        this->load_parameter();

        // Subscriber Initialization
        this->state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&GPSCompassPlannerNode::state_callback, this, std::placeholders::_1));
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::mode_callback, this, std::placeholders::_1));
        this->curr_wp_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/position/waypoint", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::curr_wp_callback, this, std::placeholders::_1));
        this->compass_success_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/compass/success", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::compass_success_callback, this, std::placeholders::_1));
        this->left_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::left_buoy_callback, this, std::placeholders::_1));
        this->right_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::right_buoy_callback, this, std::placeholders::_1));
        this->trajectory_sub = this->create_subscription<std_msgs::msg::String>("/robot/trajectory", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::trajectory_callback, this, std::placeholders::_1));
        this->surface_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/surface", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::surface_callback, this, std::placeholders::_1));
        this->underwater_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::underwater_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&GPSCompassPlannerNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->setmode_pub = this->create_publisher<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS());
        this->gps_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS());
        this->compass_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS());
        this->vision_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS());
        this->mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS());
        this->left_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS());
        this->right_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(20ms, std::bind(&GPSCompassPlannerNode::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&GPSCompassPlannerNode::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "PLANNER → node started");
    }

private:
    void load_parameter() {
        this->phase_duration.clear();
        this->phase_duration["0"] = this->get_parameter("phase_duration.0").as_double();
        this->phase_duration["1"] = this->get_parameter("phase_duration.1").as_double();
        this->phase_duration["2"] = this->get_parameter("phase_duration.2").as_double();
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
    }

    void compass_success_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->compass_success = msg->data;
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

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->trajectory = msg->data;
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
            this->is_swap = false;
            this->nav_start_time = rclcpp::Time();
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

    void swap_buoy_colors() {
        std::swap(this->left_buoy, this->right_buoy);
        RCLCPP_INFO(this->get_logger(), "PLANNER → L-Buoy: '%s', R-Buoy: '%s'", this->left_buoy.c_str(), this->right_buoy.c_str());
    }

    void handle_waypoint_0(double current_time) {
        // Waypoint 0: Floating Ball - Vision (with timer) → GPS
        this->mission = "Floating Ball";

        if (this->nav_start_time.seconds() == 0) {
            this->nav_start_time = this->now();
        }

        try {
            double phase_duration = this->get_phase_duration(std::to_string(this->current_waypoint));
            double elapsed_time = current_time - this->nav_start_time.seconds();

            if (elapsed_time >= phase_duration) {
                this->gps_navigation();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 0: Floating Ball - GPS");
            } else {
                this->vision_navigation();
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 0: Floating Ball - Vision");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 0: %s", e.what());
        }
    }

    void handle_waypoint_1(double current_time) {
        // Waypoint 1: Compass → Vision (with timer) → GPS
        if (!this->compass_success) {
            this->compass_navigation();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 1: Floating Ball - Compass");
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            try {
                double phase_duration = this->get_phase_duration(std::to_string(this->current_waypoint));
                double elapsed_time = current_time - this->nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    this->gps_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 1: Floating Ball - GPS");
                } else {
                    this->vision_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 1: Floating Ball - Vision");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 1: %s", e.what());
            }
        }
    }

    void handle_waypoint_2(double current_time) {
        // Waypoint 2: Compass → Vision (with timer) → Swap buoy → Surface Imaging (Vision until surface) → Vision (reverse timer) → GPS
        if (!this->compass_success) {
            this->compass_navigation();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: Floating Ball - Compass");
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            if (!this->is_swap) {
                this->is_swap = true;
                this->swap_buoy_colors();
            }

            try {
                double vision_phase_duration = this->get_phase_duration(std::to_string(this->current_waypoint));
                double vision_elapsed_time = current_time - this->nav_start_time.seconds();

                if (vision_elapsed_time >= vision_phase_duration) {
                    this->mission = "Surface Imaging";

                    if (!this->is_surface) {
                        this->vision_navigation();
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: Surface Imaging - Vision");
                    } else {
                        if (!this->reverse_started) {
                            this->reverse_started = true;
                            this->reverse_start_time = this->now();
                        }

                        double reverse_phase_duration = this->get_phase_duration("reverse");
                        double reverse_elapsed_time = current_time - this->reverse_start_time.seconds();

                        if (reverse_elapsed_time >= reverse_phase_duration) {
                            this->mission = "Underwater Imaging";
                            this->gps_navigation();
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: Underwater Imaging - GPS");
                        } else {
                            this->vision_navigation();
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: Surface Imaging - Vision");
                        }
                    }
                } else {
                    this->vision_navigation();
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 2: Floating Ball - Vision");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 2: %s", e.what());
            }
        }
    }

    void handle_waypoint_3(double current_time) {
        // Waypoint 3: Compass → Underwater Imaging (Vision until underwater complete) → Vision (reverse timer) → GPS
        if (!this->compass_success) {
            this->compass_navigation();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 3: Underwater Imaging - Compass");
        } else {
            if (!this->is_underwater) {
                this->vision_navigation();
            } else {
                if (this->nav_start_time.seconds() == 0) {
                    this->nav_start_time = this->now();
                }

                try {
                    double phase_duration = this->get_phase_duration("reverse");
                    double elapsed_time = current_time - this->nav_start_time.seconds();

                    if (elapsed_time >= phase_duration) {
                        this->gps_navigation();
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 3: Underwater Imaging - GPS");
                    } else {
                        this->vision_navigation();
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 3: Underwater Imaging - Vision");
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "PLANNER → Error getting phase duration for waypoint 3: %s", e.what());
                }
            }
        }
    }

    void handle_waypoint_4(double current_time) {
        // Waypoint 4: Docking - GPS only
        this->mission = "Docking";
        this->gps_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 4: Docking - GPS");
    }

    void handle_waypoint_5(double current_time) {
        // Waypoint 5: Docking - Vision navigation (road to finish)
        this->vision_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Waypoint 5: Road to Finish!");
    }

    void mission_planner() {
        std::lock_guard<std::mutex> lock(this->mutex);

        if (!this->auto_mode) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PLANNER → RC AUTO mode is disabled, waiting for RC AUTO mode...");
            return;
        }

        // Get current time in seconds
        double current_time = this->now().seconds();

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
        default:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → Invalid waypoint: %d", this->current_waypoint);
            break;
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

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PLANNER → WP:%d | Mission:%s | Compass Success:%s", this->current_waypoint, this->mission.c_str(), this->compass_success ? "✓" : "✗");
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr curr_wp_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_success_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_buoy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_buoy_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_sub;
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
    rclcpp::Time reverse_start_time;
    mavros_msgs::msg::State current_state;
    std::map<std::string, double> phase_duration;
    std::string mission;

    // Variable Initialization
    std::string trajectory = "B";
    std::string default_left = "green_buoy";
    std::string default_right = "red_buoy";
    std::string left_buoy = "green_buoy";
    std::string right_buoy = "red_buoy";
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool compass_success = false;
    bool is_swap = false;
    bool is_surface = false;
    bool is_underwater = false;
    bool reverse_started = false;
    int current_waypoint = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSCompassPlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
