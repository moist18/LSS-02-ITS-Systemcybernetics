#define _USE_MATH_DEFINES
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

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

class CompassNavigation : public rclcpp::Node {
public:
    CompassNavigation()
        : Node("compass_navigation") {

        // Parameter Declaration
        this->declare_parameter<int>("compass.heading.A.count", 3);
        this->declare_parameter<int>("compass.heading.B.count", 3);

        int count_A = this->get_parameter("compass.heading.A.count").as_int();
        int count_B = this->get_parameter("compass.heading.B.count").as_int();

        for (int i = 1; i <= count_A; ++i) {
            this->declare_parameter<double>("compass.heading.A." + std::to_string(i), 0.0);
        }

        for (int i = 1; i <= count_B; ++i) {
            this->declare_parameter<double>("compass.heading.B." + std::to_string(i), 0.0);
        }

        // RC Parameter Declaration
        this->declare_parameter<int>("compass.rc.min_value", 1300);
        this->declare_parameter<int>("compass.rc.mid_value", 1500);
        this->declare_parameter<int>("compass.rc.max_value", 1700);
        this->declare_parameter<double>("compass.tolerance", 10.0);

        // PID Parameter Declaration
        this->declare_parameter<double>("compass.pid.yaw.kp", 3.0);
        this->declare_parameter<double>("compass.pid.yaw.ki", 0.0);
        this->declare_parameter<double>("compass.pid.yaw.kd", 0.3);
        this->declare_parameter<double>("compass.pid.yaw.min_output", -1.0);
        this->declare_parameter<double>("compass.pid.yaw.max_output", 1.0);

        // Parameter Setup
        this->load_parameter();

        // Subscriber Initialization
        this->compass_hdg_sub = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(), std::bind(&CompassNavigation::compass_hdg_callback, this, std::placeholders::_1));
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::vision_control_callback, this, std::placeholders::_1));
        this->curr_wp_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/position/waypoint", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::curr_wp_callback, this, std::placeholders::_1));
        this->trajectory_sub = this->create_subscription<std_msgs::msg::String>("/robot/trajectory", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::trajectory_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&CompassNavigation::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->rc_override_pub = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", rclcpp::ServicesQoS());
        this->compass_success_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/compass/success", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(50ms, std::bind(&CompassNavigation::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&CompassNavigation::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "COMPASS-NAV → Node started");
    }

private:
    void load_parameter() {
        this->target_heading_A.clear();
        this->target_heading_B.clear();

        // Heading Parameter Initialization
        int count_A = this->get_parameter("compass.heading.A.count").as_int();
        for (int i = 1; i <= count_A; ++i) {
            std::string key = std::to_string(i);
            this->target_heading_A[key] = this->get_parameter("compass.heading.A." + key).as_double();
        }

        int count_B = this->get_parameter("compass.heading.B.count").as_int();
        for (int i = 1; i <= count_B; ++i) {
            std::string key = std::to_string(i);
            this->target_heading_B[key] = this->get_parameter("compass.heading.B." + key).as_double();
        }

        // RC Parameter Initialization
        this->rc_min_value = this->get_parameter("compass.rc.min_value").as_int();
        this->rc_mid_value = this->get_parameter("compass.rc.mid_value").as_int();
        this->rc_max_value = this->get_parameter("compass.rc.max_value").as_int();
        this->heading_tolerance = this->get_parameter("compass.tolerance").as_double();

        // PID Parameter Initialization
        double yaw_kp = this->get_parameter("compass.pid.yaw.kp").as_double();
        double yaw_ki = this->get_parameter("compass.pid.yaw.ki").as_double();
        double yaw_kd = this->get_parameter("compass.pid.yaw.kd").as_double();
        double yaw_min = this->get_parameter("compass.pid.yaw.min_output").as_double();
        double yaw_max = this->get_parameter("compass.pid.yaw.max_output").as_double();

        // Initialize PID controller
        this->yaw_pid = std::make_unique<PID>(this->get_clock(), yaw_kp, yaw_ki, yaw_kd, yaw_min, yaw_max);
    }

    void param_callback() {
        this->load_parameter();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "COMPASS-NAV → Parameters Reloaded!");
    }

    void compass_hdg_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_heading = fmod(msg->data, 360.0);
        if (this->current_heading < 0.0) {
            this->current_heading += 360.0;
        }
    }

    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->auto_mode = msg->data;
    }

    void gps_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->gps_control = msg->data;
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->compass_control = msg->data;
    }

    void vision_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->vision_control = msg->data;
    }

    void curr_wp_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_waypoint = msg->data;
        this->compass_success = false;
        if (this->yaw_pid) {
            this->yaw_pid->reset();
        }
    }

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->trajectory = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->compass_success = false;
            if (this->yaw_pid) {
                this->yaw_pid->reset();
            }
            RCLCPP_INFO(this->get_logger(), "COMPASS-NAV → Successfully Reset Mission!");
        }
    }

    double get_heading_error(double target_heading) {
        double error = target_heading - this->current_heading;

        // Normalize to [-180, 180]
        if (error > 180.0) {
            error -= 360.0;
        } else if (error < -180.0) {
            error += 360.0;
        }

        return error;
    }

    void override_control(int yaw, int pitch, int throttle, int lateral) {
        mavros_msgs::msg::OverrideRCIn rc_override;
        rc_override.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);
        rc_override.channels[0] = static_cast<uint16_t>(yaw);
        rc_override.channels[1] = static_cast<uint16_t>(pitch);
        rc_override.channels[2] = static_cast<uint16_t>(throttle);
        rc_override.channels[3] = static_cast<uint16_t>(lateral);
        this->rc_override_pub->publish(rc_override);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "COMPASS-NAV → RC Y:%d", yaw);
    }

    bool compass_navigation(double target_heading, double tolerance) {
        target_heading = fmod(target_heading, 360.0);
        if (target_heading < 0.0) {
            target_heading += 360.0;
        }

        double error = this->get_heading_error(target_heading);

        if (fabs(error) <= tolerance) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "COMPASS-NAV → Target heading reached! Current: %.1f°", this->current_heading);
            return true;
        }

        // Compute PID output (-1.0 to 1.0)
        double yaw_control = this->yaw_pid->compute(error);

        // Map PID output to RC PWM values
        int yaw_output;
        
        if (yaw_control > 0.0) {
            // Turn right: 1500 to max
            yaw_output = this->rc_mid_value + static_cast<int>(yaw_control * (this->rc_max_value - this->rc_mid_value));
        } else {
            // Turn left: min to 1500
            yaw_output = this->rc_mid_value + static_cast<int>(yaw_control * (this->rc_mid_value - this->rc_min_value));
        }

        // Clamp to operational limits
        yaw_output = std::max(this->rc_min_value, std::min(this->rc_max_value, yaw_output));

        // Publish RC override
        this->override_control(yaw_output, 1500, 1500, 1500);

        return false;
    }

    void runner() {
        std::lock_guard<std::mutex> lock(this->mutex);

        const auto& target_heading_map = (this->trajectory == "A") ? this->target_heading_A : this->target_heading_B;
        std::string wp_key = std::to_string(this->current_waypoint);

        if (target_heading_map.find(wp_key) != target_heading_map.end() && this->current_waypoint > 0) {
            double target_heading = target_heading_map.at(wp_key);

            if (this->auto_mode) {
                if (!this->gps_control && this->compass_control && !this->vision_control) {
                    if (!this->compass_success) {
                        if (this->compass_navigation(target_heading, this->heading_tolerance)) {
                            this->compass_success = true;
                            RCLCPP_INFO(this->get_logger(), "COMPASS-NAV → Compass Navigation Completed!");
                        }
                    }
                }
            }
        }

        // Publish compass success status only on state change
        if (this->compass_success != this->prev_compass_success) {
            auto success_msg = std_msgs::msg::Bool();
            success_msg.data = this->compass_success;
            this->compass_success_pub->publish(success_msg);
            this->prev_compass_success = this->compass_success;
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_hdg_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr curr_wp_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compass_success_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Mutex Declaration
    std::mutex mutex;

    // PID Controller
    std::unique_ptr<PID> yaw_pid;

    // Variable Declaration
    std::map<std::string, double> target_heading_A;
    std::map<std::string, double> target_heading_B;

    // RC Values
    int rc_min_value = 1200;
    int rc_mid_value = 1500;
    int rc_max_value = 1800;

    // Variable Initialization
    std::string trajectory = "B";
    double current_heading = 0.0;
    double heading_tolerance = 10.0;
    int current_waypoint = 0;
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool compass_success = false;
    bool prev_compass_success = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompassNavigation>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
