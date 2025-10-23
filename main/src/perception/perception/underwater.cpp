#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class UnderwaterNode : public rclcpp::Node {
public:
    UnderwaterNode()
        : Node("underwater_node") {

        // Parameter Declaration
        this->declare_parameter<int>("camera_device", 1);

        // Parameter Setup
        this->camera_device = this->get_parameter("camera_device").as_int();

        // Subscriber Initialization
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::vision_control_callback, this, std::placeholders::_1));
        this->lidar_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/lidar", rclcpp::SensorDataQoS(), std::bind(&UnderwaterNode::lidar_callback, this, std::placeholders::_1));
        this->mission_sub = this->create_subscription<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::mission_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->underwater_image_pub = this->create_publisher<sensor_msgs::msg::Image>("/robot/image/underwater", rclcpp::ServicesQoS());
        this->underwater_mission_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS());
        this->mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(50ms, std::bind(&UnderwaterNode::runner, this));

        RCLCPP_INFO(this->get_logger(), "Underwater node started");
    }

private:
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

    void lidar_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->lidar = msg->data;
    }

    void mission_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->mission = msg->data;
    }

    void reset_mission_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->mission = "";
            this->is_underwater = false;
            RCLCPP_INFO(this->get_logger(), "U-VIZ → Successfully Reset Mission!");
        }
    }

    // Processor method
    void processor(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(this->mutex);

        if (this->auto_mode) {
            if (!this->gps_control && !this->compass_control && this->vision_control) {
                if (this->mission == "Underwater Imaging") {
                    if (this->lidar && !this->is_underwater) {
                        this->is_underwater = true;
                        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                        this->underwater_image_pub->publish(*img_msg);
                        RCLCPP_INFO(this->get_logger(), "U-VIZ: Underwater Imaging!");
                    }
                }
            }
        }
    }

    // Runner method
    void runner() {
        static bool camera_initialized = false;

        if (!camera_initialized) {
            this->cam.open(this->camera_device);
            if (!this->cam.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "U-VIZ → Failed to open camera /dev/video%d", this->camera_device);
                return;
            }
            this->cam.set(cv::CAP_PROP_FRAME_WIDTH, this->WIDTH);
            this->cam.set(cv::CAP_PROP_FRAME_HEIGHT, this->HEIGHT);
            RCLCPP_INFO(this->get_logger(), "U-VIZ → Camera opened successfully: /dev/video%d", this->camera_device);
            camera_initialized = true;
        }

        cv::Mat frame;
        bool success = this->cam.read(frame);

        if (success && !frame.empty()) {
            processor(frame);
        }

        // Publish underwater mission status
        {
            std::lock_guard<std::mutex> lock(this->mutex);
            auto underwater_msg = std_msgs::msg::Bool();
            underwater_msg.data = this->is_underwater;
            this->underwater_mission_pub->publish(underwater_msg);
        }
    }

    // Subscriber Declaration
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gps_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr compass_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_control_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr underwater_image_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr underwater_mission_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Constant Initialization
    int WIDTH = 640;
    int HEIGHT = 480;

    // Variable Initialization
    int camera_device;
    cv::VideoCapture cam;
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool lidar = false;
    bool is_underwater = false;
    std::string mission = "";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnderwaterNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}