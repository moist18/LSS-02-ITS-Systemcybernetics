#define _USE_MATH_DEFINES
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class CompassNavigation : public rclcpp::Node {
public:
    CompassNavigation()
        : Node("compass_navigation")
        , prev_time(this->now()) {

        // Parameter Declaration for trajectory counts
        this->declare_parameter<int>("compass.heading.A.count", 3);
        this->declare_parameter<int>("compass.heading.B.count", 3);

        // Dynamically declare parameters based on counts
        int count_A = this->get_parameter("compass.heading.A.count").as_int();
        int count_B = this->get_parameter("compass.heading.B.count").as_int();

        for (int i = 1; i <= count_A; ++i) {
            this->declare_parameter<double>("compass.heading.A." + std::to_string(i), 0.0);
        }

        for (int i = 1; i <= count_B; ++i) {
            this->declare_parameter<double>("compass.heading.B." + std::to_string(i), 0.0);
        }

        // PID and control parameters
        this->declare_parameter<double>("compass.min_speed", 2.0);
        this->declare_parameter<double>("compass.max_speed", 10.0);
        this->declare_parameter<double>("compass.tolerance", 10.0);
        this->declare_parameter<double>("compass.pid.kp", 1.0);
        this->declare_parameter<double>("compass.pid.ki", 0.0);
        this->declare_parameter<double>("compass.pid.kd", 0.0);

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
        this->local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", rclcpp::ServicesQoS());
        this->compass_success_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/compass/success", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(20ms, std::bind(&CompassNavigation::runner, this));
        this->param_timer = this->create_wall_timer(1s, std::bind(&CompassNavigation::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "Compass Navigation node started");
    }

private:
    void load_parameter() {
        this->target_heading_A.clear();
        this->target_heading_B.clear();

        // Dynamically load trajectory A headings
        int count_A = this->get_parameter("compass.heading.A.count").as_int();
        for (int i = 1; i <= count_A; ++i) {
            std::string key = std::to_string(i);
            this->target_heading_A[key] = this->get_parameter("compass.heading.A." + key).as_double();
        }

        // Dynamically load trajectory B headings
        int count_B = this->get_parameter("compass.heading.B.count").as_int();
        for (int i = 1; i <= count_B; ++i) {
            std::string key = std::to_string(i);
            this->target_heading_B[key] = this->get_parameter("compass.heading.B." + key).as_double();
        }

        // Load control parameters
        this->min_rotation_speed = this->get_parameter("compass.min_speed").as_double();
        this->max_rotation_speed = this->get_parameter("compass.max_speed").as_double();
        this->heading_tolerance = this->get_parameter("compass.tolerance").as_double();
        this->kp = this->get_parameter("compass.pid.kp").as_double();
        this->ki = this->get_parameter("compass.pid.ki").as_double();
        this->kd = this->get_parameter("compass.pid.kd").as_double();

        RCLCPP_INFO(this->get_logger(), "COMPASS-NAV → Parameters loaded | A:%d B:%d | Tolerance: %.1f°, Kp: %.2f", count_A, count_B, this->heading_tolerance, this->kp);
    }

    void param_callback() {
        this->load_parameter();
        RCLCPP_INFO(this->get_logger(), "COMPASS-NAV → Parameters Reloaded!");
    }

    void compass_hdg_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_heading = fmod(msg->data, 360.0);
        if (this->current_heading < 0.0) {
            this->current_heading += 360.0;
        }

        // Reset smoothed heading if discontinuity detected
        double diff = this->smoothed_heading - this->current_heading;
        // Normalize to [-180, 180]
        if (diff > 180.0) {
            diff -= 360.0;
        } else if (diff < -180.0) {
            diff += 360.0;
        }
        // Safety check for extreme discontinuity (should never trigger after normalization)
        if (fabs(diff) > 180.0) {
            this->smoothed_heading = this->current_heading;
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
        this->heading_error_integral = 0.0;
        this->prev_heading_error = 0.0;
    }

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->trajectory = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->compass_success = false;
            this->heading_error_integral = 0.0;
            this->smoothed_heading = this->current_heading;
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

    double smooth_heading(double target_heading) {
        auto current_time = this->now();
        double dt = (current_time - this->prev_time).seconds();

        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "COMPASS-NAV → Time delta anomaly: %.2fs, resetting PID", dt);
            this->prev_time = current_time;
            this->heading_error_integral = 0.0;
            this->prev_heading_error = 0.0;
            this->smoothed_heading = this->current_heading;
            return this->current_heading;
        }

        double error = this->get_heading_error(target_heading);

        // Gradual integral decay when within tolerance, accumulation when outside
        if (fabs(error) < this->heading_tolerance) {
            this->heading_error_integral *= 0.95;
        } else {
            // Update integral with anti-windup
            this->heading_error_integral += error * dt;

            // Clamp integral
            if (this->heading_error_integral > 45.0) {
                this->heading_error_integral = 45.0;
            } else if (this->heading_error_integral < -45.0) {
                this->heading_error_integral = -45.0;
            }
        }

        // Calculate derivative
        double error_derivative = (error - this->prev_heading_error) / dt;
        double max_change = this->max_rotation_speed * dt;

        // PID control signal
        double control_signal = this->kp * error + this->ki * this->heading_error_integral + this->kd * error_derivative;

        // Clamp control signal
        if (control_signal > max_change) {
            control_signal = max_change;
        } else if (control_signal < -max_change) {
            control_signal = -max_change;
        }

        // Ensure minimum rotation speed
        if (fabs(error) > this->heading_tolerance && fabs(control_signal) < this->min_rotation_speed * dt) {
            control_signal = (control_signal >= 0 ? 1.0 : -1.0) * this->min_rotation_speed * dt;
        }

        // Update smoothed heading
        this->smoothed_heading = fmod(this->smoothed_heading + control_signal, 360.0);
        if (this->smoothed_heading < 0.0) {
            this->smoothed_heading += 360.0;
        }

        this->prev_heading_error = error;
        this->prev_time = current_time;

        return this->smoothed_heading;
    }

    bool compass_navigation(double target_heading, double tolerance) {
        target_heading = fmod(target_heading, 360.0);
        if (target_heading < 0.0) {
            target_heading += 360.0;
        }

        double smoothed_target = this->smooth_heading(target_heading);
        double error = this->get_heading_error(target_heading);

        if (fabs(error) <= tolerance) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "COMPASS-NAV → Target heading reached! Current: %.1f°", this->current_heading);
            return true;
        }

        // Publish pose setpoint
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.stamp = this->now();
        pose.header.frame_id = "base_link";
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;

        // Convert heading to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, smoothed_target * M_PI / 180.0);
        pose.pose.orientation = tf2::toMsg(q);

        this->local_pos_pub->publish(pose);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "COMPASS-NAV → Current: %.1f°, Target: %.1f°, Error: %.1f°", this->current_heading, smoothed_target, error);

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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compass_success_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Variable Declaration
    std::map<std::string, double> target_heading_A;
    std::map<std::string, double> target_heading_B;
    rclcpp::Time prev_time;

    // Variable Initialization
    std::string trajectory = "B";
    double current_heading = 0.0;
    double smoothed_heading = 0.0;
    double prev_heading_error = 0.0;
    double heading_error_integral = 0.0;
    double min_rotation_speed = 2.0;
    double max_rotation_speed = 10.0;
    double heading_tolerance = 10.0;
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
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
