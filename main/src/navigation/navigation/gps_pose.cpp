#include <cmath>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct WayPoint {
    double latitude;
    double longitude;
    double altitude;
};

class GPSNavigation : public rclcpp::Node {
public:
    GPSNavigation()
        : Node("gps_navigation") {
        // Parameters
        this->declare_parameter<std::string>("waypoints_A", "");
        this->declare_parameter<std::string>("waypoints_B", "");
        this->declare_parameter<double>("radius", 2.0);

        // Subscribers
        global_pos_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", 10,
            std::bind(&GPSNavigation::global_pos_callback, this, std::placeholders::_1));

        mode_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/mode", 10, std::bind(&GPSNavigation::mode_callback, this, std::placeholders::_1));

        gps_control_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/robot/control/gps", 10, std::bind(&GPSNavigation::gps_control_callback, this, std::placeholders::_1));

        compass_control_sub_ = create_subscription<std_msgs::msg::Bool>("/robot/control/compass", 10,
            std::bind(&GPSNavigation::compass_control_callback, this, std::placeholders::_1));

        vision_control_sub_ = create_subscription<std_msgs::msg::Bool>("/robot/control/vision", 10,
            std::bind(&GPSNavigation::vision_control_callback, this, std::placeholders::_1));

        trajectory_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot/trajectory", 10, std::bind(&GPSNavigation::trajectory_callback, this, std::placeholders::_1));

        reset_mission_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/reset", 10, std::bind(&GPSNavigation::reset_mission_callback, this, std::placeholders::_1));

        // Publishers
        geopose_pub_ = create_publisher<geographic_msgs::msg::GeoPoseStamped>("mavros/setpoint_position/global", 10);
        curr_wp_pub_ = create_publisher<std_msgs::msg::Int32>("/robot/position/waypoint", 10);

        // Timers
        timer_ = create_wall_timer(20ms, std::bind(&GPSNavigation::run, this));
        param_timer_ = create_wall_timer(1s, std::bind(&GPSNavigation::param_callback, this));

        load_parameters();
    }

private:
    std::mutex mutex_;
    sensor_msgs::msg::NavSatFix current_position_;
    int current_waypoint_ = 0;
    bool auto_mode_ = true;
    bool gps_control_ = false;
    bool compass_control_ = false;
    bool vision_control_ = false;
    bool is_finish_ = false;
    std::string trajectory_ = "B";
    std::vector<WayPoint> waypoints_A_;
    std::vector<WayPoint> waypoints_B_;
    double radius_ = 2.0;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub_;

    // Publishers
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr curr_wp_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr param_timer_;

    void load_parameters() {
        std::lock_guard<std::mutex> lock(mutex_);

        // Load waypoints
        waypoints_A_ = parse_waypoints(get_parameter("waypoints_A").as_string());
        waypoints_B_ = parse_waypoints(get_parameter("waypoints_B").as_string());
        radius_ = get_parameter("radius").as_double();
    }

    std::vector<WayPoint> parse_waypoints(const std::string& yaml_str) {
        std::vector<WayPoint> waypoints;
        try {
            YAML::Node config = YAML::Load(yaml_str);
            for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
                WayPoint wp;
                wp.latitude = it->second["latitude"].as<double>();
                wp.longitude = it->second["longitude"].as<double>();
                wp.altitude = it->second["altitude"].as<double>();
                waypoints.push_back(wp);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to parse waypoints: %s", e.what());
        }
        return waypoints;
    }

    void param_callback() {
        load_parameters();
        RCLCPP_INFO(get_logger(), "Parameters reloaded");
    }

    void global_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_position_ = *msg;
    }

    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto_mode_ = msg->data;
    }

    void gps_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gps_control_ = msg->data;
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        compass_control_ = msg->data;
    }

    void vision_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        vision_control_ = msg->data;
    }

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(mutex_);
            current_waypoint_ = 0;
            is_finish_ = false;
            RCLCPP_INFO(get_logger(), "NAV: Successfully Reset Mission!");
        }
    }

    double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // Earth radius in meters
        const double phi1 = lat1 * M_PI / 180.0;
        const double phi2 = lat2 * M_PI / 180.0;
        const double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        const double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        const double a = pow(sin(delta_phi / 2.0), 2) + cos(phi1) * cos(phi2) * pow(sin(delta_lambda / 2.0), 2);
        const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        return R * c;
    }

    void gps_navigation(double lat, double lon, double alt) {
        auto geopose = geographic_msgs::msg::GeoPoseStamped();
        geopose.header.stamp = now();
        geopose.header.frame_id = "map";
        geopose.pose.position.latitude = lat;
        geopose.pose.position.longitude = lon;
        geopose.pose.position.altitude = alt;

        std::lock_guard<std::mutex> lock(mutex_);
        const double current_lat = current_position_.latitude;
        const double current_lon = current_position_.longitude;
        const double desired_yaw = atan2(lat - current_lat, lon - current_lon);

        tf2::Quaternion q;
        q.setRPY(0, 0, desired_yaw);
        geopose.pose.orientation = tf2::toMsg(q);

        geopose_pub_->publish(geopose);
    }

    void run() {
        std::lock_guard<std::mutex> lock(mutex_);

        const auto& waypoints = (trajectory_ == "A") ? waypoints_A_ : waypoints_B_;

        if (!is_finish_ && !waypoints.empty()) {
            const auto& current_wp = waypoints[current_waypoint_];
            const double distance = calculate_distance(
                current_position_.latitude, current_position_.longitude, current_wp.latitude, current_wp.longitude);

            if (auto_mode_) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Current WP_%d: (%f, %f)", current_waypoint_,
                    current_wp.latitude, current_wp.longitude);

                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000, "WP Distance: %.2f m, WP Radius: %.1f m", distance, radius_);

                if (gps_control_ && !compass_control_ && !vision_control_) {
                    gps_navigation(current_wp.latitude, current_wp.longitude, current_wp.altitude);
                }

                if (distance < radius_) {
                    if (current_waypoint_ < static_cast<int>(waypoints.size()) - 1) {
                        current_waypoint_++;
                        auto wp_msg = std_msgs::msg::Int32();
                        wp_msg.data = current_waypoint_;
                        curr_wp_pub_->publish(wp_msg);

                        RCLCPP_INFO(get_logger(), "Reached Waypoint %d", current_waypoint_);
                        RCLCPP_INFO(get_logger(), "Moving to Waypoint %d", current_waypoint_ + 1);
                    } else {
                        is_finish_ = true;
                        auto wp_msg = std_msgs::msg::Int32();
                        wp_msg.data = current_waypoint_ + 1;
                        curr_wp_pub_->publish(wp_msg);
                        RCLCPP_INFO(get_logger(), "Reached last waypoint: Moving to Dock");
                    }
                }
            }
        } else if (is_finish_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "NAV_FINISH: Road to Finish!");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNavigation>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}