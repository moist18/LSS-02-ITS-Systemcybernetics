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

        // Subscriber Initialization
        this->nav_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&GPSNode::gps_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->gps_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/gps", rclcpp::ServicesQoS());
        this->cog_pub = this->create_publisher<std_msgs::msg::Float64>("/robot/cog", rclcpp::ServicesQoS());
        this->sog_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/robot/sog", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(1s, std::bind(&GPSNode::data_publisher, this));

        RCLCPP_INFO(this->get_logger(), "GPS node started");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_lat = msg->latitude;
        this->current_lon = msg->longitude;
        this->current_time = this->now();
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
        const double lon_diff = (lon2 - this->prev_lon) * M_PI / 180.0;
        const double lat2_rad = lat2 * M_PI / 180.0;
        const double prev_lat_rad = this->prev_lat * M_PI / 180.0;

        const double y = sin(lon_diff) * cos(lat2_rad);
        const double x = cos(prev_lat_rad) * sin(lat2_rad) - sin(prev_lat_rad) * cos(lat2_rad) * cos(lon_diff);

        double bearing = atan2(y, x);
        bearing = fmod((bearing * 180.0 / M_PI) + 360.0, 360.0);
        return bearing;
    }

    double calculate_sog(double lat2, double lon2, const rclcpp::Time& current_time) {
        const double distance = haversine(this->prev_lat, this->prev_lon, lat2, lon2);
        const double time_diff = (current_time - this->prev_time).seconds();

        if (time_diff > 0.0) {
            return distance / time_diff;
        }
        return 0.0;
    }

    void data_publisher() {
        std::lock_guard<std::mutex> lock(this->mutex);

        if (!std::isnan(this->current_lat) && !std::isnan(this->current_lon)) {
            // Publish GPS coordinates
            auto gps_msg = std_msgs::msg::Float32MultiArray();
            gps_msg.data = { static_cast<float>(this->current_lat), static_cast<float>(this->current_lon) };
            this->gps_pub->publish(gps_msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Latitude: %.6f, Longitude: %.6f", this->current_lat, this->current_lon);

            if (!std::isnan(this->prev_lat) && !std::isnan(this->prev_lon)) {
                // Calculate and publish COG
                double cog_value = calculate_cog(this->current_lat, this->current_lon);
                auto cog_msg = std_msgs::msg::Float64();
                cog_msg.data = cog_value;
                this->cog_pub->publish(cog_msg);

                // Calculate and publish SOG
                double sog_value = this->calculate_sog(this->current_lat, this->current_lon, this->current_time);
                double mps_value = sog_value;
                double kmh_value = mps_value * 3.6;
                double knot_value = kmh_value / 1.852;

                auto sog_data = std_msgs::msg::Float32MultiArray();
                sog_data.data = { static_cast<float>(kmh_value), static_cast<float>(knot_value) };
                this->sog_pub->publish(sog_data);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current COG: %.2f°", cog);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current SOG: %.2f km/h", sog_msg.data[0]);
            }

            // Update previous values
            this->prev_lat = this->current_lat;
            this->prev_lon = this->current_lon;
            this->prev_time = this->current_time;
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gps_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cog_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sog_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Variable Declaration
    rclcpp::Time current_time;
    rclcpp::Time prev_time;
    double current_lat = NAN;
    double current_lon = NAN;
    double prev_lat = NAN;
    double prev_lon = NAN;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}