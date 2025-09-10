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
        this->declare_parameter<double>("phase_duration.A.0", 8.0);
        this->declare_parameter<double>("phase_duration.A.1", 8.0);
        this->declare_parameter<double>("phase_duration.A.2", 8.0);
        this->declare_parameter<double>("phase_duration.A.3", 3.0);
        this->declare_parameter<double>("phase_duration.A.4", 3.0);
        this->declare_parameter<double>("phase_duration.A.5", 0.0);
        this->declare_parameter<double>("phase_duration.A.6", 0.0);
        this->declare_parameter<double>("phase_duration.B.0", 8.0);
        this->declare_parameter<double>("phase_duration.B.1", 8.0);
        this->declare_parameter<double>("phase_duration.B.2", 8.0);
        this->declare_parameter<double>("phase_duration.B.3", 3.0);
        this->declare_parameter<double>("phase_duration.B.4", 3.0);
        this->declare_parameter<double>("phase_duration.B.5", 0.0);
        this->declare_parameter<double>("phase_duration.B.6", 0.0);

        // Parameter Setup
        load_parameter();

        // Subscriber Initialization
        this->state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", rclcpp::SensorDataQoS(), std::bind(&PlannerNode::state_callback, this, std::placeholders::_1));
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&PlannerNode::mode_callback, this, std::placeholders::_1));
        this->curr_wp_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/position/waypoint", rclcpp::ServicesQoS(), std::bind(&PlannerNode::curr_wp_callback, this, std::placeholders::_1));
        this->compass_success_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/compass/success", rclcpp::ServicesQoS(), std::bind(&PlannerNode::compass_success_callback, this, std::placeholders::_1));
        this->left_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS(), std::bind(&PlannerNode::left_buoy_callback, this, std::placeholders::_1));
        this->right_buoy_sub = this->create_subscription<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS(), std::bind(&PlannerNode::right_buoy_callback, this, std::placeholders::_1));
        this->trajectory_sub = this->create_subscription<std_msgs::msg::String>("/robot/trajectory", rclcpp::ServicesQoS(), std::bind(&PlannerNode::trajectory_callback, this, std::placeholders::_1));
        this->surface_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/surface", rclcpp::ServicesQoS(), std::bind(&PlannerNode::surface_callback, this, std::placeholders::_1));
        this->underwater_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS(), std::bind(&PlannerNode::underwater_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&PlannerNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->setmode_pub = this->create_publisher<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS());
        this->gps_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS());
        this->compass_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS());
        this->vision_control_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS());
        this->mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS());
        this->left_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS());
        this->right_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(20ms, std::bind(&PlannerNode::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&PlannerNode::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "GPS Vision Planner node started");
    }

private:
    void load_parameter() {
        std::lock_guard<std::mutex> lock(this->mutex);

        // Load individual parameters instead of YAML
        this->phase_duration.clear();

        // Load trajectory A parameters
        std::map<std::string, double> traj_A;
        traj_A["0"] = this->get_parameter("phase_duration.A.0").as_double();
        traj_A["1"] = this->get_parameter("phase_duration.A.1").as_double();
        traj_A["2"] = this->get_parameter("phase_duration.A.2").as_double();
        traj_A["3"] = this->get_parameter("phase_duration.A.3").as_double();
        traj_A["4"] = this->get_parameter("phase_duration.A.4").as_double();
        traj_A["5"] = this->get_parameter("phase_duration.A.5").as_double();
        traj_A["6"] = this->get_parameter("phase_duration.A.6").as_double();
        this->phase_duration["A"] = traj_A;

        // Load trajectory B parameters
        std::map<std::string, double> traj_B;
        traj_B["0"] = this->get_parameter("phase_duration.B.0").as_double();
        traj_B["1"] = this->get_parameter("phase_duration.B.1").as_double();
        traj_B["2"] = this->get_parameter("phase_duration.B.2").as_double();
        traj_B["3"] = this->get_parameter("phase_duration.B.3").as_double();
        traj_B["4"] = this->get_parameter("phase_duration.B.4").as_double();
        traj_B["5"] = this->get_parameter("phase_duration.B.5").as_double();
        traj_B["6"] = this->get_parameter("phase_duration.B.6").as_double();
        this->phase_duration["B"] = traj_B;
    }

    void param_callback() {
        this->load_parameter();
        RCLCPP_INFO(this->get_logger(), "Parameters reloaded");
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
            RCLCPP_INFO(this->get_logger(), "PLANNER: Successfully Reset Mission!");
        }
    }

    double get_phase_duration(const std::string& traj, const std::string& waypoint) {
        if (this->phase_duration.find(traj) != this->phase_duration.end() && this->phase_duration[traj].find(waypoint) != this->phase_duration[traj].end()) {
            return this->phase_duration[traj][waypoint];
        }
        return 0.0;
    }

    void set_navigation_control(const std::string& mode, bool gps, bool compass, bool vision) {
        if (this->current_state.mode != mode) {
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = mode;
            this->setmode_pub->publish(mode_msg);
        }

        std::lock_guard<std::mutex> lock(this->mutex);
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
    }

    void gps_navigation() {
        this->set_navigation_control("GUIDED", true, false, false);
    }

    void compass_navigation() {
        this->set_navigation_control("GUIDED", false, true, false);
    }

    void vision_navigation() {
        this->set_navigation_control("MANUAL", false, false, true);
    }

    void swap_buoy_colors() {
        std::lock_guard<std::mutex> lock(this->mutex);
        std::swap(this->left_buoy, this->right_buoy);
        RCLCPP_INFO(this->get_logger(), "L-Buoy: '%s', R-Buoy: '%s'", this->left_buoy.c_str(), this->right_buoy.c_str());
    }

    void handle_waypoint_0(double current_time) {
        this->mission = "Floating Ball";

        if (this->nav_start_time.seconds() == 0) {
            this->nav_start_time = rclcpp::Time::max();
            this->nav_start_time = this->now();
        }

        try {
            double phase_duration = this->get_phase_duration(this->trajectory, std::to_string(this->current_waypoint));
            double elapsed_time = current_time - this->nav_start_time.seconds();

            if (elapsed_time >= phase_duration) {
                this->gps_navigation();
            } else {
                this->vision_navigation();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error getting phase duration for waypoint 0: %s", e.what());
        }
    }

    void handle_waypoint_1(double current_time) {
        if (!this->compass_success) {
            this->compass_navigation();
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            try {
                double phase_duration = this->get_phase_duration(this->trajectory, std::to_string(this->current_waypoint));
                double elapsed_time = current_time - this->nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    if (!this->is_swap) {
                        this->is_swap = true;
                        this->swap_buoy_colors();
                    }
                    this->gps_navigation();
                } else {
                    this->vision_navigation();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error getting phase duration for waypoint 1: %s", e.what());
            }
        }
    }

    void handle_waypoint_2(double current_time) {
        if (!this->compass_success) {
            this->compass_navigation();
        } else {
            if (this->nav_start_time.seconds() == 0) {
                this->nav_start_time = this->now();
            }

            try {
                double phase_duration = this->get_phase_duration(this->trajectory, std::to_string(this->current_waypoint));
                double elapsed_time = current_time - this->nav_start_time.seconds();

                if (elapsed_time >= phase_duration) {
                    this->mission = "Surface Imaging";
                    this->gps_navigation();
                } else {
                    this->vision_navigation();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error getting phase duration for waypoint 2: %s", e.what());
            }
        }
    }

    void handle_waypoint_3(double current_time) {
        if (!this->compass_success) {
            this->compass_navigation();
        } else {
            if (!this->is_surface) {
                this->vision_navigation();
            } else {
                if (this->nav_start_time.seconds() == 0) {
                    this->nav_start_time = this->now();
                }

                try {
                    double phase_duration = this->get_phase_duration(this->trajectory, std::to_string(this->current_waypoint));
                    double elapsed_time = current_time - this->nav_start_time.seconds();

                    if (elapsed_time >= phase_duration) {
                        this->mission = "Underwater Imaging";
                        this->gps_navigation();
                    } else {
                        this->vision_navigation();
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error getting phase duration for waypoint 3: %s", e.what());
                }
            }
        }
    }

    void handle_waypoint_4(double current_time) {
        if (!this->compass_success) {
            this->compass_navigation();
        } else {
            if (!this->is_underwater) {
                this->vision_navigation();
            } else {
                if (this->nav_start_time.seconds() == 0) {
                    this->nav_start_time = this->now();
                }

                try {
                    double phase_duration = this->get_phase_duration(this->trajectory, std::to_string(this->current_waypoint));
                    double elapsed_time = current_time - this->nav_start_time.seconds();

                    if (elapsed_time >= phase_duration) {
                        this->mission = "Docking";
                        this->gps_navigation();
                    } else {
                        this->vision_navigation();
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error getting phase duration for waypoint 4: %s", e.what());
                }
            }
        }
    }

    void handle_waypoint_5() {
        this->gps_navigation();
    }

    void handle_waypoint_6() {
        this->vision_navigation();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PLANNER_FINISH: Road to Finish!");
    }

    void mission_planner() {
        std::lock_guard<std::mutex> lock(this->mutex);
        auto now = this->now();

        if (!this->auto_mode) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RC AUTO mode is disabled, waiting for RC AUTO mode...");
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
            this->handle_waypoint_5();
            break;
        case 6:
            this->handle_waypoint_6();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid waypoint: %d", this->current_waypoint);
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

        auto mission_msg = std_msgs::msg::String();
        mission_msg.data = this->mission;
        this->mission_pub->publish(mission_msg);

        auto left_msg = std_msgs::msg::String();
        left_msg.data = this->left_buoy;
        this->left_buoy_pub->publish(left_msg);

        auto right_msg = std_msgs::msg::String();
        right_msg.data = this->right_buoy;
        this->right_buoy_pub->publish(right_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current mission: %s", this->mission.c_str());
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
    mavros_msgs::msg::State current_state;
    std::map<std::string, std::map<std::string, double>> phase_duration;
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
    int current_waypoint = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}