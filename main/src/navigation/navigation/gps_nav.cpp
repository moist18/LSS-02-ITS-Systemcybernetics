#include <cmath>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

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

        // Parameter Declaration
        this->declare_parameter<double>("navigation.radius", 2.0);
        this->declare_parameter<int>("navigation.waypoints_A.count", 0);
        this->declare_parameter<int>("navigation.waypoints_B.count", 0);

        for (int i = 0; i < 10; ++i) {
            this->declare_parameter<double>("navigation.waypoints_A.waypoint_" + std::to_string(i) + ".latitude", 0.0);
            this->declare_parameter<double>("navigation.waypoints_A.waypoint_" + std::to_string(i) + ".longitude", 0.0);
            this->declare_parameter<double>("navigation.waypoints_A.waypoint_" + std::to_string(i) + ".altitude", 0.0);

            this->declare_parameter<double>("navigation.waypoints_B.waypoint_" + std::to_string(i) + ".latitude", 0.0);
            this->declare_parameter<double>("navigation.waypoints_B.waypoint_" + std::to_string(i) + ".longitude", 0.0);
            this->declare_parameter<double>("navigation.waypoints_B.waypoint_" + std::to_string(i) + ".altitude", 0.0);
        }

        // Subscriber Initialization
        this->global_pos_sub = create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&GPSNavigation::global_pos_callback, this, std::placeholders::_1));
        this->mode_sub = create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::vision_control_callback, this, std::placeholders::_1));
        this->trajectory_sub = create_subscription<std_msgs::msg::String>("/robot/trajectory", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::trajectory_callback, this, std::placeholders::_1));
        this->reset_mission_sub = create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&GPSNavigation::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->geopose_pub = create_publisher<geographic_msgs::msg::GeoPoseStamped>("mavros/setpoint_position/global", rclcpp::ServicesQoS());
        this->curr_wp_pub = create_publisher<std_msgs::msg::Int32>("/robot/position/waypoint", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = create_wall_timer(20ms, std::bind(&GPSNavigation::runner, this));
        this->param_timer = create_wall_timer(1s, std::bind(&GPSNavigation::param_callback, this));

        // Parameter Setup
        load_parameter();
        RCLCPP_INFO(this->get_logger(), "GPS Navigation node started");
    }

private:
    std::vector<WayPoint> load_waypoints(const std::string& trajectory) {
        std::vector<WayPoint> waypoints;
        std::string prefix = "navigation.waypoints_" + trajectory;

        try {
            // Get the number of waypoints
            int count = get_parameter(prefix + ".count").as_int();

            // Load each waypoint
            for (int i = 0; i < count; ++i) {
                std::string wp_prefix = prefix + ".waypoint_" + std::to_string(i);
                WayPoint wp;
                wp.latitude = get_parameter(wp_prefix + ".latitude").as_double();
                wp.longitude = get_parameter(wp_prefix + ".longitude").as_double();
                wp.altitude = get_parameter(wp_prefix + ".altitude").as_double();
                waypoints.push_back(wp);
            }
        } catch (const rclcpp::ParameterTypeException& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load waypoints for trajectory %s: %s", trajectory.c_str(), e.what());
        }

        return waypoints;
    }

    void load_parameter() {
        std::lock_guard<std::mutex> lock(this->mutex);

        // Load parameters using ROS2 parameter system
        this->radius = get_parameter("navigation.radius").as_double();
        this->waypoints_A = load_waypoints("A");
        this->waypoints_B = load_waypoints("B");

        RCLCPP_INFO(get_logger(), "Parameters loaded - Radius: %.2f, Waypoints A: %zu, Waypoints B: %zu", this->radius, this->waypoints_A.size(), this->waypoints_B.size());
    }

    void param_callback() {
        load_parameter();
        RCLCPP_INFO(get_logger(), "Parameters reloaded");
    }

    void global_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_position = *msg;
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

    void trajectory_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->trajectory = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->current_waypoint = 0;
            this->is_finish = false;
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

        std::lock_guard<std::mutex> lock(this->mutex);
        const double current_lat = this->current_position.latitude;
        const double current_lon = this->current_position.longitude;
        const double desired_yaw = atan2(lat - current_lat, lon - current_lon);

        tf2::Quaternion q;
        q.setRPY(0, 0, desired_yaw);
        geopose.pose.orientation = tf2::toMsg(q);

        this->geopose_pub->publish(geopose);
    }

    void runner() {
        std::lock_guard<std::mutex> lock(this->mutex);

        const auto& waypoints = (this->trajectory == "A") ? this->waypoints_A : this->waypoints_B;

        if (!this->is_finish && !waypoints.empty()) {
            const auto& current_wp = waypoints[this->current_waypoint];
            const double distance = calculate_distance(this->current_position.latitude, this->current_position.longitude, current_wp.latitude, current_wp.longitude);

            if (this->auto_mode) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Current WP_%d: (%f, %f)", this->current_waypoint, current_wp.latitude, current_wp.longitude);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WP Distance: %.2f m, WP Radius: %.1f m", distance, this->radius);

                if (this->gps_control && !this->compass_control && !this->vision_control) {
                    gps_navigation(current_wp.latitude, current_wp.longitude, current_wp.altitude);
                }

                if (distance < this->radius) {
                    if (this->current_waypoint < static_cast<int>(waypoints.size()) - 1) {
                        this->current_waypoint++;
                        auto wp_msg = std_msgs::msg::Int32();
                        wp_msg.data = this->current_waypoint;
                        this->curr_wp_pub->publish(wp_msg);

                        RCLCPP_INFO(get_logger(), "Reached Waypoint %d", this->current_waypoint);
                        RCLCPP_INFO(get_logger(), "Moving to Waypoint %d", this->current_waypoint + 1);
                    } else {
                        this->is_finish = true;
                        auto wp_msg = std_msgs::msg::Int32();
                        wp_msg.data = this->current_waypoint + 1;
                        this->curr_wp_pub->publish(wp_msg);
                        RCLCPP_INFO(get_logger(), "Reached last waypoint: Moving to Dock");
                    }
                }
            }
        } else if (this->is_finish) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "NAV_FINISH: Road to Finish!");
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr curr_wp_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Variable Declaration
    sensor_msgs::msg::NavSatFix current_position;
    std::vector<WayPoint> waypoints_A;
    std::vector<WayPoint> waypoints_B;

    // Variable Initialization
    std::string trajectory = "B";
    double radius = 2.0;
    int current_waypoint = 0;
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool is_finish = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNavigation>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}