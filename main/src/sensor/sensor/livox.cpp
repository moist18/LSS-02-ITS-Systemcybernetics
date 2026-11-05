#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class LivoxNode : public rclcpp::Node {
public:
    LivoxNode()
        : Node("livox_node") {

        // Parameter Declaration
        this->declare_parameter<double>("livox.min_range", 0.5);
        this->declare_parameter<double>("livox.max_range", 5.0);
        this->declare_parameter<int>("livox.min_point", 10);
        this->declare_parameter<double>("livox.min_angle", 345.0);
        this->declare_parameter<double>("livox.max_angle", 15.0);
        this->declare_parameter<double>("livox.min_height", -0.5);
        this->declare_parameter<double>("livox.max_height", 0.5);

        // Parameter Setup
        load_parameter();

        // Subscriber Initialization
        this->cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&LivoxNode::cloud_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->lidar_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/lidar", 1);

        // Rate Timer Initialization
        this->param_timer = this->create_wall_timer(1s, std::bind(&LivoxNode::param_callback, this));

        RCLCPP_INFO(this->get_logger(), "LIVOX → node started");
    }

private:
    void load_parameter() {
        this->min_range   = this->get_parameter("livox.min_range").as_double();
        this->max_range   = this->get_parameter("livox.max_range").as_double();
        this->min_point   = this->get_parameter("livox.min_point").as_int();
        this->min_angle   = this->get_parameter("livox.min_angle").as_double();
        this->max_angle   = this->get_parameter("livox.max_angle").as_double();
        this->min_height  = this->get_parameter("livox.min_height").as_double();
        this->max_height  = this->get_parameter("livox.max_height").as_double();
    }

    void param_callback() {
        this->load_parameter();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "LIVOX → Parameters Reloaded!");
    }

    bool angle_in_range(double angle_deg) {
        // Handles wrap-around range (e.g. 345° → 15°)
        return (this->min_angle > this->max_angle)
            ? (angle_deg >= this->min_angle || angle_deg <= this->max_angle)
            : (angle_deg >= this->min_angle && angle_deg <= this->max_angle);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);

        int valid_count = 0;
        double last_angle = 0.0;

        // Iterate over PointCloud2 fields (x, y, z)
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            // Skip NaN or Inf points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            // Filter by height (z-axis)
            if (z < this->min_height || z > this->max_height)
                continue;

            // Compute horizontal range (distance in XY plane)
            const double range = std::sqrt(x * x + y * y);

            // Skip if range not valid
            if (range <= this->min_range || range > this->max_range)
                continue;

            // Compute horizontal angle in degrees [0, 360)
            double angle_deg = std::atan2(y, x) * 180.0 / M_PI;
            if (angle_deg < 0.0)
                angle_deg += 360.0;

            // Skip if angle not in configured range
            if (!this->angle_in_range(angle_deg))
                continue;

            valid_count++;
            last_angle = angle_deg;
        }

        // Publish detection result
        std_msgs::msg::Bool detection_msg;
        detection_msg.data = valid_count >= this->min_point;
        this->lidar_pub->publish(detection_msg);

        if (detection_msg.data) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "LIVOX → Object detected | Points:%d Angle:%.1f Range:%.2f-%.2fm Height:%.2f-%.2fm",
                valid_count, last_angle, this->min_range, this->max_range,
                this->min_height, this->max_height);
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr param_timer;

    // Variable Declaration
    std::mutex mutex;
    double min_range;
    double max_range;
    int    min_point;
    double min_angle;
    double max_angle;
    double min_height;
    double max_height;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
