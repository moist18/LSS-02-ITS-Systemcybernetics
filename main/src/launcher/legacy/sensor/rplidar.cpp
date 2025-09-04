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

        // Parameter Declaration
        this->declare_parameter<double>("min_range", 0.5);
        this->declare_parameter<double>("max_range", 2.0);
        this->declare_parameter<int>("min_point", 5);
        this->declare_parameter<int>("min_angle", 345);
        this->declare_parameter<int>("max_angle", 15);

        // Subscriber Initialization
        this->laser_sub = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LaserScannerNode::laser_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->lidar_pub = create_publisher<std_msgs::msg::Bool>("/robot/lidar", 1);

        // Rate Timer Initialization
        this->param_timer = create_wall_timer(1s, std::bind(&LaserScannerNode::param_callback, this));

        load_parameter();
    }

private:
    void load_parameter() {
        this->min_valid_range = get_parameter("min_range").as_double();
        this->max_valid_range = get_parameter("max_range").as_double();
        this->min_valid_point = get_parameter("min_point").as_int();
        this->min_angle = get_parameter("min_angle").as_int();
        this->max_angle = get_parameter("max_angle").as_int();
    }

    void param_callback() {
        load_parameter();
    }

    bool angle_in_range(int angle_deg) {
        return (this->min_angle > this->max_angle) ? (angle_deg >= this->min_angle || angle_deg <= this->max_angle) : (angle_deg >= this->min_angle && angle_deg <= this->max_angle);
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
            if (range <= this->min_valid_range || range > this->max_valid_range)
                continue;

            valid_count++;
            last_angle = normalized_angle;
        }

        // Publish detection result
        std_msgs::msg::Bool detection_msg;
        detection_msg.data = valid_count >= this->min_valid_point;
        this->lidar_pub->publish(detection_msg);

        if (detection_msg.data) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "STOP: Object detected in angle %d within %.2f-%.2fm", last_angle, this->min_valid_range, this->max_valid_range);
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

    // Publisher Declaration
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr param_timer;

    // Variable Declaration
    double min_valid_range;
    double max_valid_range;
    int min_valid_point;
    int min_angle;
    int max_angle;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}