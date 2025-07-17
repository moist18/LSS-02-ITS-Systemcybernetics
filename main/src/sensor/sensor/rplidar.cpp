#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class LaserScannerNode : public rclcpp::Node {
public:
    LaserScannerNode()
        : Node("lidar_node") {
        // Parameter declaration
        declare_parameter<double>("min_range", 0.5);
        declare_parameter<double>("max_range", 2.0);
        declare_parameter<int>("min_point", 5);
        declare_parameter<int>("min_angle", 345);
        declare_parameter<int>("max_angle", 15);

        update_parameters();

        // Publisher and Subscriber
        lidar_pub_ = create_publisher<std_msgs::msg::Bool>("/robot/lidar", 1);
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScannerNode::laser_callback, this, std::placeholders::_1));

        // Parameter update timer
        param_timer_ = create_wall_timer(1s, std::bind(&LaserScannerNode::param_callback, this));
    }

private:
    void update_parameters() {
        min_valid_range_ = get_parameter("min_range").as_double();
        max_valid_range_ = get_parameter("max_range").as_double();
        min_valid_point_ = get_parameter("min_point").as_int();
        min_angle_ = get_parameter("min_angle").as_int();
        max_angle_ = get_parameter("max_angle").as_int();
    }

    void param_callback() { update_parameters(); }

    bool angle_in_range(int angle_deg) {
        return (min_angle_ > max_angle_) ? (angle_deg >= min_angle_ || angle_deg <= max_angle_)
                                         : (angle_deg >= min_angle_ && angle_deg <= max_angle_);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int valid_count = 0;
        int last_angle = 0;

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const float range = msg->ranges[i];
            const float angle_rad = msg->angle_min + i * msg->angle_increment;

            // Convert to normalized degrees
            float angle_deg = (angle_rad * 180.0 / M_PI) + 180.0;
            angle_deg = fmod(angle_deg, 360.0);
            const int normalized_angle = static_cast<int>(angle_deg);

            if (!angle_in_range(normalized_angle))
                continue;
            if (std::isinf(range) || std::isnan(range))
                continue;
            if (range <= min_valid_range_ || range > max_valid_range_)
                continue;

            valid_count++;
            last_angle = normalized_angle;
        }

        // Publish detection result
        std_msgs::msg::Bool detection_msg;
        detection_msg.data = valid_count >= min_valid_point_;
        lidar_pub_->publish(detection_msg);

        if (detection_msg.data) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "STOP: Object detected in angle %d within %.2f-%.2fm", last_angle, min_valid_range_, max_valid_range_);
        }
    }

    // Parameters
    double min_valid_range_;
    double max_valid_range_;
    int min_valid_point_;
    int min_angle_;
    int max_angle_;

    // ROS2 components
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr param_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}