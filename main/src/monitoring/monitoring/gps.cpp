#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

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

        // Timer for publishing
        timer_ = this->create_wall_timer(1s, std::bind(&GPSNode::data_publisher, this));
    }

private:
    std::mutex mutex_;
    double current_lat_ = NAN;
    double current_lon_ = NAN;
    rclcpp::Time current_time_;

    double prev_lat_ = NAN;
    double prev_lon_ = NAN;
    rclcpp::Time prev_time_;

    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gps_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cog_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sog_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        current_time_ = this->now();
    }

    double haversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // Earth radius in meters
        const double phi1 = lat1 * M_PI / 180.0;
        const double phi2 = lat2 * M_PI / 180.0;
        const double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        const double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

        const double a = pow(sin(delta_phi / 2.0), 2) + cos(phi1) * cos(phi2) * pow(sin(delta_lambda / 2.0), 2);
        const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        return R * c;
    }

    double calculate_cog(double lat2, double lon2) {
        const double lon_diff = (lon2 - prev_lon_) * M_PI / 180.0;
        const double lat2_rad = lat2 * M_PI / 180.0;
        const double prev_lat_rad = prev_lat_ * M_PI / 180.0;

        const double y = sin(lon_diff) * cos(lat2_rad);
        const double x = cos(prev_lat_rad) * sin(lat2_rad) - sin(prev_lat_rad) * cos(lat2_rad) * cos(lon_diff);

        double bearing = atan2(y, x);
        bearing = fmod((bearing * 180.0 / M_PI) + 360.0, 360.0);
        return bearing;
    }

    double calculate_sog(double lat2, double lon2, const rclcpp::Time& current_time) {
        const double distance = haversine(prev_lat_, prev_lon_, lat2, lon2);
        const double time_diff = (current_time - prev_time_).seconds();

        if (time_diff > 0.0) {
            return distance / time_diff;
        }
        return 0.0;
    }

    void data_publisher() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!std::isnan(current_lat_) && !std::isnan(current_lon_)) {
            // Publish GPS coordinates
            auto gps_msg = std_msgs::msg::Float32MultiArray();
            gps_msg.data = { static_cast<float>(current_lat_), static_cast<float>(current_lon_) };
            gps_pub_->publish(gps_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Latitude: %.6f, Longitude: %.6f",
                current_lat_, current_lon_);

            if (!std::isnan(prev_lat_) && !std::isnan(prev_lon_)) {
                // Calculate and publish COG
                const double cog = calculate_cog(current_lat_, current_lon_);
                auto cog_msg = std_msgs::msg::Float64();
                cog_msg.data = cog;
                cog_pub_->publish(cog_msg);

                // Calculate and publish SOG
                const double sog_mps = calculate_sog(current_lat_, current_lon_, current_time_);
                auto sog_msg = std_msgs::msg::Float32MultiArray();
                sog_msg.data = { static_cast<float>(sog_mps * 3.6), // km/h
                    static_cast<float>(sog_mps * 1.94384) }; // knots
                sog_pub_->publish(sog_msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current COG: %.2f°", cog);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "Current SOG: %.2f km/h", sog_msg.data[0]);
            }

            // Update previous values
            prev_lat_ = current_lat_;
            prev_lon_ = current_lon_;
            prev_time_ = current_time_;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}