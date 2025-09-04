#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class SurfaceNode : public rclcpp::Node {
public:
    SurfaceNode()
        : Node("surface_node") {

        // Subscriber Initialization
        this->mode_sub = create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::vision_control_callback, this, std::placeholders::_1));
        this->mission_sub = create_subscription<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::mission_callback, this, std::placeholders::_1));
        this->reset_mission_sub = create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&SurfaceNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->surface_image_pub = create_publisher<sensor_msgs::msg::Image>("/robot/image/surface", rclcpp::ServicesQoS());
        this->surface_mission_pub = create_publisher<std_msgs::msg::Bool>("/robot/mission/surface", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = create_wall_timer(50ms, std::bind(&SurfaceNode::runner, this));

        RCLCPP_INFO(get_logger(), "Surface cam node started");
    }

private:
    void mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->auto_mode = msg->data;
    }

    void gps_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->gps_control = msg->data;
    }

    void compass_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->compass_control = msg->data;
    }

    void vision_control_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->vision_control = msg->data;
    }

    void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
        this->mission = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            this->mission = "";
            this->is_surface = false;
            RCLCPP_INFO(get_logger(), "S-VIZ: Successfully Reset Mission!");
        }
    }

    // Processor method
    void processor(cv::Mat& frame) {
        if (this->auto_mode) {
            // Surface detection logic - placeholder for now
            if (this->gps_control && !this->compass_control && !this->vision_control) {
                if (this->mission == "Surface Detection") {
                    this->is_surface = true;

                    // Convert OpenCV Mat to ROS Image message
                    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    this->surface_image_pub->publish(*img_msg);
                }
            }
        }
    }

    // Runner method
    void runner() {
        static bool camera_initialized = false;

        if (!camera_initialized) {
            this->cam.open(0); // Use camera 0 for surface detection
            if (!this->cam.isOpened()) {
                RCLCPP_ERROR(get_logger(), "Failed to open camera");
                return;
            }
            this->cam.set(cv::CAP_PROP_FRAME_WIDTH, this->WIDTH);
            this->cam.set(cv::CAP_PROP_FRAME_HEIGHT, this->HEIGHT);
            camera_initialized = true;
        }

        cv::Mat frame;
        bool success = this->cam.read(frame);

        if (success && !frame.empty()) {
            processor(frame);
        }

        // Publish surface mission status
        auto surface_msg = std_msgs::msg::Bool();
        surface_msg.data = this->is_surface;
        this->surface_mission_pub->publish(surface_msg);
    }

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr surface_image_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr surface_mission_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Constant Initialization
    int WIDTH = 640;
    int HEIGHT = 480;

    // Variable Initialization
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool is_surface = false;
    std::string mission = "";
    cv::VideoCapture cam;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SurfaceNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}