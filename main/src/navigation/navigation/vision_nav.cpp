#include <array>
#include <chrono>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

using namespace std::chrono_literals;

class PID {
public:
    PID(rclcpp::Clock::SharedPtr clock, double kp, double ki, double kd, double min_output, double max_output)
        : clock(clock)
        , kp(kp)
        , ki(ki)
        , kd(kd)
        , min_output(min_output)
        , max_output(max_output) {
        reset();
    }

    void reset() {
        auto current_time = this->clock->now().nanoseconds() / 1e9;
        this->prev_time = current_time;
        this->prev_error = 0.0;
        this->integral = 0.0;
    }

    double compute(double error) {
        auto current_time = this->clock->now().nanoseconds() / 1e9;
        double dt = std::max(current_time - this->prev_time, 1e-6);

        this->integral += error * dt;
        double derivative = (error - this->prev_error) / dt;

        double output = this->kp * error + this->ki * this->integral + this->kd * derivative;
        output = std::max(this->min_output, std::min(this->max_output, output));

        this->prev_error = error;
        this->prev_time = current_time;

        return output;
    }

private:
    rclcpp::Clock::SharedPtr clock;
    double kp, ki, kd;
    double min_output, max_output;
    double prev_time, prev_error, integral;
};

class VisionNavigation : public rclcpp::Node {
public:
    VisionNavigation()
        : Node("vision_navigation") {

        // RC Parameter Declaration
        this->declare_parameter<std::vector<int64_t>>("vision.rc.throttle", std::vector<int64_t> { 2000, 1700 });
        this->declare_parameter<std::vector<int64_t>>("vision.rc.min_value", std::vector<int64_t> { 1000, 1200 });
        this->declare_parameter<int>("vision.rc.mid_value", 1500);
        this->declare_parameter<std::vector<int64_t>>("vision.rc.max_value", std::vector<int64_t> { 2000, 1800 });

        // Docking Timer Parameter Declaration
        this->declare_parameter<double>("vision.timer.docking", 5.0);

        // PID Parameter Declaration
        this->declare_parameter<double>("vision.pid.yaw.kp", 1.0);
        this->declare_parameter<double>("vision.pid.yaw.ki", 0.0);
        this->declare_parameter<double>("vision.pid.yaw.kd", 0.1);
        this->declare_parameter<double>("vision.pid.yaw.min_output", -1.0);
        this->declare_parameter<double>("vision.pid.yaw.max_output", 1.0);

        // Parameter Setup
        this->load_parameter();

        // Subscriber Initialization
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::vision_control_callback, this, std::placeholders::_1));
        this->lidar_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/lidar", rclcpp::SensorDataQoS(), std::bind(&VisionNavigation::lidar_callback, this, std::placeholders::_1));
        this->mission_sub = this->create_subscription<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::mission_callback, this, std::placeholders::_1));
        this->surface_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/surface", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::surface_callback, this, std::placeholders::_1));
        this->underwater_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::underwater_callback, this, std::placeholders::_1));
        this->raw_yaw_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/vision/raw_yaw", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::raw_yaw_callback, this, std::placeholders::_1));
        this->obj_detection_sub = this->create_subscription<std_msgs::msg::Bool>("/detect/object/bool", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::obj_detection_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&VisionNavigation::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->rc_override_pub = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(50ms, std::bind(&VisionNavigation::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&VisionNavigation::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "Vision Navigation node started");
    }

private:
    void load_parameter() {
        // RC Parameter Initialization
        this->rc_channels_default.fill(65535);

        auto throttle_param = this->get_parameter("vision.rc.throttle").as_integer_array();
        this->rc_throttle = { static_cast<int>(throttle_param[0]), static_cast<int>(throttle_param[1]) };

        auto min_value_param = this->get_parameter("vision.rc.min_value").as_integer_array();
        this->min_rc_value = { static_cast<int>(min_value_param[0]), static_cast<int>(min_value_param[1]) };

        this->mid_rc_value = this->get_parameter("vision.rc.mid_value").as_int();

        auto max_value_param = this->get_parameter("vision.rc.max_value").as_integer_array();
        this->max_rc_value = { static_cast<int>(max_value_param[0]), static_cast<int>(max_value_param[1]) };

        // Docking Timer Parameter Initialization
        this->dock_duration = this->get_parameter("vision.timer.docking").as_double();

        // PID Parameter Initialization
        double yaw_kp = this->get_parameter("vision.pid.yaw.kp").as_double();
        double yaw_ki = this->get_parameter("vision.pid.yaw.ki").as_double();
        double yaw_kd = this->get_parameter("vision.pid.yaw.kd").as_double();
        double yaw_min = this->get_parameter("vision.pid.yaw.min_output").as_double();
        double yaw_max = this->get_parameter("vision.pid.yaw.max_output").as_double();

        this->pid_yaw = std::make_unique<PID>(this->get_clock(), yaw_kp, yaw_ki, yaw_kd, yaw_min, yaw_max);
    }

    void param_callback() {
        this->load_parameter();
    }

    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->auto_mode = msg->data;
    }

    void gps_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->gps_control = msg->data;
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->compass_control = msg->data;
    }

    void vision_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->vision_control = msg->data;
    }

    void lidar_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->lidar = msg->data;
    }

    void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
        this->mission = msg->data;
    }

    void surface_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->is_surface = msg->data;
    }

    void underwater_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->is_underwater = msg->data;
    }

    void raw_yaw_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        this->raw_yaw = msg->data;
    }

    void obj_detection_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->objects_detected = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            this->mission = "";
            this->is_surface = false;
            this->is_finish = false;
            this->dock_timer_started = false;
            RCLCPP_INFO(this->get_logger(), "V-NAV: Successfully Reset Mission!");
        }
    }

    int calculate_rc_value(double pid_output) {
        int mission_index = (this->mission == "Floating Ball") ? 0 : 1;
        int rc_value = this->mid_rc_value + pid_output * (this->max_rc_value[mission_index] - this->mid_rc_value);
        return static_cast<int>(std::max(this->min_rc_value[mission_index], std::min(this->max_rc_value[mission_index], rc_value)));
    }

    void override_control(int yaw, int pitch, int throttle, int lateral) {
        mavros_msgs::msg::OverrideRCIn rc_override;
        rc_override.channels = this->rc_channels_default;
        rc_override.channels[0] = static_cast<uint16_t>(yaw);
        rc_override.channels[1] = static_cast<uint16_t>(pitch);
        rc_override.channels[2] = static_cast<uint16_t>(throttle);
        rc_override.channels[3] = static_cast<uint16_t>(lateral);
        this->rc_override_pub->publish(rc_override);
        RCLCPP_INFO(this->get_logger(), "YAW: %d", yaw);
    }

    void runner() {
        int rc_yaw = this->mid_rc_value;
        int rc_throttle = 1500;

        if (this->objects_detected) {
            int mission_index = (this->mission == "Floating Ball") ? 0 : 1;
            double yaw_error = static_cast<double>(this->raw_yaw - this->mid_rc_value) / (this->max_rc_value[mission_index] - this->mid_rc_value);
            rc_yaw = this->calculate_rc_value(this->pid_yaw->compute(yaw_error));
            rc_throttle = this->rc_throttle[mission_index];
            RCLCPP_INFO(get_logger(), "RC YAW: %d, Raw Yaw: %d, Error: %f", rc_yaw, this->raw_yaw, yaw_error);
        } else {
            rc_yaw = this->mid_rc_value;
            int mission_index = (this->mission == "Floating Ball") ? 0 : 1;
            rc_throttle = this->rc_throttle[mission_index];
            this->pid_yaw->reset();
        }

        if (this->auto_mode) {
            if (!this->gps_control && !this->compass_control && this->vision_control) {
                if (this->mission == "Surface Imaging") {
                    if (this->lidar && !this->is_surface) {
                        this->is_surface = true;
                        rc_yaw = this->mid_rc_value;
                        rc_throttle = this->mid_rc_value - 200;
                        this->pid_yaw->reset();
                    } else if (this->is_surface) {
                        rc_yaw = this->mid_rc_value;
                        rc_throttle = this->mid_rc_value - 200;
                        this->pid_yaw->reset();
                    }
                } else if (this->mission == "Underwater Imaging") {
                    if (this->lidar && !this->is_underwater) {
                        rc_yaw = this->mid_rc_value;
                        rc_throttle = this->mid_rc_value - 200;
                        this->pid_yaw->reset();
                    } else if (this->is_underwater) {
                        rc_yaw = this->mid_rc_value;
                        rc_throttle = this->mid_rc_value - 200;
                        this->pid_yaw->reset();
                    }
                } else if (this->mission == "Docking") {
                    if (!this->is_finish) {
                        this->is_finish = true;
                        this->dock_timer_started = true;
                        this->dock_start_time = this->now();
                    }
                }

                if (!this->is_finish) {
                    this->override_control(rc_yaw, 1500, rc_throttle, 1500);
                } else {
                    // Timer-based finish logic
                    auto current_time = this->now();
                    double elapsed_time = (current_time - this->dock_start_time).seconds();

                    if (elapsed_time >= this->dock_duration) {
                        this->override_control(1500, 1500, 1500, 1500);
                        RCLCPP_INFO(this->get_logger(), "VISION_FINISH: FINISH WOYYY!!!");
                    } else {
                        this->override_control(rc_yaw, 1500, rc_throttle, 1500);
                        RCLCPP_INFO(this->get_logger(), "VISION_FINISH: Road to Finish!");
                    }
                }
            }
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr surface_mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr underwater_mission_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr raw_yaw_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obj_detection_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Variable Declaration
    std::array<uint16_t, 18> rc_channels_default;
    std::vector<int> rc_throttle;
    std::vector<int> min_rc_value;
    int mid_rc_value;
    std::vector<int> max_rc_value;
    std::unique_ptr<PID> pid_yaw;

    // Timer Variables
    rclcpp::Time dock_start_time;
    double dock_duration;

    // Variable Initialization
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool lidar = false;
    bool is_surface = false;
    bool is_underwater = false;
    bool is_finish = false;
    bool objects_detected = false;
    int raw_yaw = 1500;
    std::string mission = "";
    bool dock_timer_started = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNavigation>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}