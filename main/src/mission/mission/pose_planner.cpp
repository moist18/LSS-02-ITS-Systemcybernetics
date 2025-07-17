#include <chrono>
#include <map>
#include <mavros_msgs/msg/state.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode()
        : Node("planner_node") {
        // Parameter Declaration - using individual parameters instead of nested maps
        this->declare_parameter("phase_duration.A.normal", 30.0);
        this->declare_parameter("phase_duration.A.compass", 15.0);
        this->declare_parameter("phase_duration.A.vision", 20.0);
        this->declare_parameter("phase_duration.B.normal", 30.0);
        this->declare_parameter("phase_duration.B.compass", 15.0);
        this->declare_parameter("phase_duration.B.vision", 20.0);

        // Subscriber Initialization
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, std::bind(&PlannerNode::state_callback, this, std::placeholders::_1));
        mode_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/mode", 10, std::bind(&PlannerNode::mode_callback, this, std::placeholders::_1));
        curr_wp_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/robot/position/waypoint", 10, std::bind(&PlannerNode::curr_wp_callback, this, std::placeholders::_1));
        compass_success_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/compass/success", 10,
            std::bind(&PlannerNode::compass_success_callback, this, std::placeholders::_1));
        left_buoy_sub = this->create_subscription<std_msgs::msg::String>(
            "/detect/left_buoy", 10, std::bind(&PlannerNode::left_buoy_callback, this, std::placeholders::_1));
        right_buoy_sub = this->create_subscription<std_msgs::msg::String>(
            "/detect/right_buoy", 10, std::bind(&PlannerNode::right_buoy_callback, this, std::placeholders::_1));
        trajectory_sub = this->create_subscription<std_msgs::msg::String>(
            "/robot/trajectory", 10, std::bind(&PlannerNode::trajectory_callback, this, std::placeholders::_1));
        surface_mission_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/mission/surface", 10, std::bind(&PlannerNode::surface_callback, this, std::placeholders::_1));
        underwater_mission_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/robot/mission/underwater", 10, std::bind(&PlannerNode::underwater_callback, this, std::placeholders::_1));
        reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/mission/reset", 10, std::bind(&PlannerNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        setmode_pub = this->create_publisher<std_msgs::msg::String>("/robot/setmode", 10);
        gps_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/gps", 10);
        compass_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/compass", 10);
        vision_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/vision", 10);
        mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", 10);
        left_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", 10);
        right_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", 10);

        // Timer Initialization
        loop_timer = this->create_wall_timer(20ms, std::bind(&PlannerNode::run, this));
        param_timer = this->create_wall_timer(1s, std::bind(&PlannerNode::param_callback, this));
    }

private:
    void param_callback() {
        std::lock_guard<std::mutex> lock(mutex);

        // Load individual parameters instead of YAML
        phase_duration.clear();

        // Load trajectory A parameters
        std::map<std::string, double> traj_A;
        traj_A["normal"] = this->get_parameter("phase_duration.A.normal").as_double();
        traj_A["compass"] = this->get_parameter("phase_duration.A.compass").as_double();
        traj_A["vision"] = this->get_parameter("phase_duration.A.vision").as_double();
        phase_duration["A"] = traj_A;

        // Load trajectory B parameters
        std::map<std::string, double> traj_B;
        traj_B["normal"] = this->get_parameter("phase_duration.B.normal").as_double();
        traj_B["compass"] = this->get_parameter("phase_duration.B.compass").as_double();
        traj_B["vision"] = this->get_parameter("phase_duration.B.vision").as_double();
        phase_duration["B"] = traj_B;

        RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        current_state = *msg;
    }

    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        auto_mode = msg->data;
    }

    void curr_wp_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        current_waypoint = msg->data;
        nav_start_time = rclcpp::Time();
    }

    void compass_success_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        compass_success = msg->data;
    }

    void left_buoy_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        default_left = msg->data;
        left_buoy = msg->data;
    }

    void right_buoy_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        default_right = msg->data;
        right_buoy = msg->data;
    }

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        trajectory = msg->data;
    }

    void surface_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        is_surface = msg->data;
    }

    void underwater_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        is_underwater = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(mutex);
            current_waypoint = 0;
            is_swap = false;
            nav_start_time = rclcpp::Time();
            mission = "";
            left_buoy = default_left;
            right_buoy = default_right;
            RCLCPP_INFO(this->get_logger(), "PLANNER: Successfully Reset Mission!");
        }
    }

    void set_navigation_control(const std::string& mode, bool gps, bool compass, bool vision) {
        if (current_state.mode != mode) {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = mode;
            setmode_pub->publish(mode_msg);
        }

        std::lock_guard<std::mutex> lock(mutex);
        gps_control = gps;
        compass_control = compass;
        vision_control = vision;

        auto gps_msg = std_msgs::msg::Bool();
        gps_msg.data = gps;
        gps_control_pub->publish(gps_msg);

        auto compass_msg = std_msgs::msg::Bool();
        compass_msg.data = compass;
        compass_control_pub->publish(compass_msg);

        auto vision_msg = std_msgs::msg::Bool();
        vision_msg.data = vision;
        vision_control_pub->publish(vision_msg);
    }

    void gps_navigation() { set_navigation_control("GUIDED", true, false, false); }

    void compass_navigation() { set_navigation_control("GUIDED", false, true, false); }

    void vision_navigation() { set_navigation_control("MANUAL", false, false, true); }

    void mission_planner() {
        std::lock_guard<std::mutex> lock(mutex);
        auto now = this->now();

        if (!auto_mode) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 5000, "RC AUTO mode is disabled, waiting for RC AUTO mode...");
            return;
        }

        // Get current time in seconds
        double current_time = now.seconds();

        switch (current_waypoint) {
        case 0:
            handle_waypoint_0(current_time);
            break;
        case 1:
            handle_waypoint_1(current_time);
            break;
        case 2:
            handle_waypoint_2(current_time);
            break;
        case 3:
            handle_waypoint_3(current_time);
            break;
        case 4:
            handle_waypoint_4();
            break;
        case 5:
            handle_waypoint_5();
            break;
        default:
            RCLCPP_WARN(get_logger(), "Invalid waypoint: %d", current_waypoint);
        }
    }

    void handle_waypoint_0(double current_time) {
        mission = "Floating Ball";

        if (nav_start_time.seconds() == 0) {
            nav_start_time = rclcpp::Time::max();
            nav_start_time = this->now();
        }

        try {
            double phase_duration = get_phase_duration(trajectory, std::to_string(current_waypoint));
            double elapsed_time = current_time - nav_start_time.seconds();

            if (elapsed_time >= phase_duration) {
                gps_navigation();
            } else {
                vision_navigation();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error getting phase duration for waypoint 0: %s", e.what());
        }
    }

    void handle_waypoint_1(double current_time) {
        if (!compass_success) {
            compass_navigation();
        } else {
            if (nav_start_time.seconds() == 0) {
                nav_start_time = this->now();
            }

            try {
                double phase_duration = get_phase_duration(trajectory, std::to_string(current_waypoint));
                double elapsed_time = current_time - nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    if (!is_swap) {
                        is_swap = true;
                        swap_buoy_colors();
                    }
                    gps_navigation();
                } else {
                    vision_navigation();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error getting phase duration for waypoint 1: %s", e.what());
            }
        }
    }

    void handle_waypoint_2(double current_time) {
        if (!compass_success) {
            compass_navigation();
        } else {
            if (mission == "Floating Ball") {
                if (nav_start_time.seconds() == 0) {
                    nav_start_time = this->now();
                }

                try {
                    double phase_duration = get_phase_duration(trajectory, std::to_string(current_waypoint));
                    double elapsed_time = current_time - nav_start_time.seconds();

                    if (elapsed_time >= phase_duration) {
                        mission = "Surface Imaging";
                        nav_start_time = rclcpp::Time();
                    } else {
                        vision_navigation();
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Error getting phase duration for waypoint 2: %s", e.what());
                }
            } else if (mission == "Surface Imaging") {
                if (!is_surface) {
                    vision_navigation();
                } else {
                    if (nav_start_time.seconds() == 0) {
                        nav_start_time = this->now();
                    }

                    try {
                        double phase_duration = get_phase_duration(trajectory, std::to_string(current_waypoint));
                        double elapsed_time = current_time - nav_start_time.seconds();

                        if (elapsed_time >= phase_duration) {
                            mission = "Underwater Imaging";
                        } else {
                            vision_navigation();
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "Error getting phase duration for waypoint 2: %s", e.what());
                    }
                }
            } else if (mission == "Underwater Imaging") {
                gps_navigation();
            }
        }
    }

    void handle_waypoint_3(double current_time) {
        if (!compass_success) {
            compass_navigation();
        } else {
            if (!is_underwater) {
                vision_navigation();
            } else {
                if (nav_start_time.seconds() == 0) {
                    nav_start_time = this->now();
                }

                try {
                    double phase_duration = get_phase_duration(trajectory, std::to_string(current_waypoint));
                    double elapsed_time = current_time - nav_start_time.seconds();

                    if (elapsed_time >= phase_duration) {
                        mission = "Docking";
                        gps_navigation();
                    } else {
                        vision_navigation();
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "Error getting phase duration for waypoint 3: %s", e.what());
                }
            }
        }
    }

    void handle_waypoint_4() { gps_navigation(); }

    void handle_waypoint_5() {
        vision_navigation();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "PLANNER_FINISH: Road to Finish!");
    }

    double get_phase_duration(const std::string& traj, const std::string& waypoint) {
        // Default phase durations matching ROS 1 reference
        std::map<std::string, std::map<std::string, double>> default_phase_duration = {
            { "A",
                { { "0", 8.0 }, { "1", 8.0 }, { "2", 8.0 }, { "3", 3.0 }, { "4", 3.0 }, { "5", 0.0 }, { "6", 0.0 } } },
            { "B",
                { { "0", 8.0 }, { "1", 8.0 }, { "2", 8.0 }, { "3", 3.0 }, { "4", 3.0 }, { "5", 0.0 }, { "6", 0.0 } } }
        };

        if (phase_duration.find(traj) != phase_duration.end()
            && phase_duration[traj].find(waypoint) != phase_duration[traj].end()) {
            return phase_duration[traj][waypoint];
        }

        if (default_phase_duration.find(traj) != default_phase_duration.end()
            && default_phase_duration[traj].find(waypoint) != default_phase_duration[traj].end()) {
            return default_phase_duration[traj][waypoint];
        }

        return 0.0;
    }

    void swap_buoy_colors() {
        std::lock_guard<std::mutex> lock(mutex);
        std::swap(left_buoy, right_buoy);
        RCLCPP_INFO(get_logger(), "L-Buoy: '%s', R-Buoy: '%s'", left_buoy.c_str(), right_buoy.c_str());
    }

    bool check_state() {
        std::lock_guard<std::mutex> lock(mutex);

        if (!current_state.connected) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000, "FCU is not connected, waiting for FCU connection...");
            return false;
        }

        if (!current_state.armed) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Robot is DISARMED, waiting for ARMING command...");
            return false;
        }

        if (current_state.mode != "GUIDED" && current_state.mode != "MANUAL") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Robot is in %s mode, waiting for GUIDED or MANUAL mode...", current_state.mode.c_str());
            return false;
        }

        return true;
    }

    void run() {
        if (!check_state()) {
            return;
        }

        mission_planner();

        auto mission_msg = std_msgs::msg::String();
        mission_msg.data = mission;
        mission_pub->publish(mission_msg);

        auto left_msg = std_msgs::msg::String();
        left_msg.data = left_buoy;
        left_buoy_pub->publish(left_msg);

        auto right_msg = std_msgs::msg::String();
        right_msg.data = right_buoy;
        right_buoy_pub->publish(right_msg);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Current mission: %s", mission.c_str());
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

    // Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Variables Declaration
    std::mutex mutex;
    rclcpp::Time nav_start_time;
    mavros_msgs::msg::State current_state;
    std::map<std::string, std::map<std::string, double>> phase_duration;
    std::string mission;

    // Variables Initialization
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
    int current_waypoint = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}