#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class GPSNode : public rclcpp::Node {
public:
    GPSNode()
        : Node("gps_node") {
        // Subscriber
        nav_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", 10, std::bind(&GPSNode::gps_callback, this, std::placeholders::_1));

        // Publishers
        gps_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/gps", 10);
        cog_pub_ = this->create_publisher<std_msgs::msg::Float64>("/robot/cog", 10);
        sog_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/sog", 10);

        // Timer for publishing GPS data
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GPSNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "GPS node started");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        current_time_ = this->now();
        has_current_fix_ = true;
    }

    double haversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // Earth radius in meters
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        double a = sin(delta_phi / 2) * sin(delta_phi / 2)
            + cos(phi1) * cos(phi2) * sin(delta_lambda / 2) * sin(delta_lambda / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c;
    }

    double calculate_cog(double lat2, double lon2) {
        double y = sin((lon2 - prev_lon_) * M_PI / 180.0) * cos(lat2 * M_PI / 180.0);
        double x = cos(prev_lat_ * M_PI / 180.0) * sin(lat2 * M_PI / 180.0)
            - sin(prev_lat_ * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * cos((lon2 - prev_lon_) * M_PI / 180.0);
        double bearing = atan2(y, x);
        return fmod((bearing * 180.0 / M_PI + 360.0), 360.0); // Convert to degrees [0, 360)
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!has_current_fix_) {
            return;
        }

        // Publish current GPS position
        auto gps_msg = std_msgs::msg::Float32MultiArray();
        gps_msg.data = { static_cast<float>(current_lat_), static_cast<float>(current_lon_) };
        gps_pub_->publish(gps_msg);

        // Calculate and publish COG and SOG if we have previous position
        if (has_prev_fix_) {
            double time_diff = (current_time_ - prev_time_).seconds();

            if (time_diff > 0) {
                // Calculate distance and speed
                double distance = haversine(prev_lat_, prev_lon_, current_lat_, current_lon_);
                double speed_ms = distance / time_diff; // m/s
                double speed_kmh = speed_ms * 3.6; // km/h
                double speed_knots = speed_ms * 1.94384; // knots

                // Calculate course over ground (COG)
                double cog = calculate_cog(current_lat_, current_lon_);

                // Publish COG
                auto cog_msg = std_msgs::msg::Float64();
                cog_msg.data = cog;
                cog_pub_->publish(cog_msg);

                // Publish SOG (Speed Over Ground) - [m/s, km/h, knots]
                auto sog_msg = std_msgs::msg::Float32MultiArray();
                sog_msg.data
                    = { static_cast<float>(speed_ms), static_cast<float>(speed_kmh), static_cast<float>(speed_knots) };
                sog_pub_->publish(sog_msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "GPS: Lat=%.6f, Lon=%.6f, COG=%.1f°, SOG=%.2f m/s", current_lat_, current_lon_, cog, speed_ms);
            }
        }

        // Update previous values
        prev_lat_ = current_lat_;
        prev_lon_ = current_lon_;
        prev_time_ = current_time_;
        has_prev_fix_ = true;
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cog_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sog_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    double current_lat_ = 0.0;
    double current_lon_ = 0.0;
    rclcpp::Time current_time_;
    bool has_current_fix_ = false;

    double prev_lat_ = 0.0;
    double prev_lon_ = 0.0;
    rclcpp::Time prev_time_;
    bool has_prev_fix_ = false;

    std::mutex mutex_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}