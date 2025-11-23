#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

class UnderwaterNode : public rclcpp::Node {
public:
    UnderwaterNode()
        : Node("underwater_node") {

        // Parameter Declaration
        this->declare_parameter<int>("config.camera_device", 2);
        this->declare_parameter<int>("config.width", 640);
        this->declare_parameter<int>("config.height", 480);
        this->declare_parameter<int>("config.fps", 30);
        this->declare_parameter<bool>("config.debug", false);
        this->declare_parameter<bool>("config.use_lidar", true);

        // Parameter Setup
        this->camera_device = this->get_parameter("config.camera_device").as_int();
        this->WIDTH = this->get_parameter("config.width").as_int();
        this->HEIGHT = this->get_parameter("config.height").as_int();
        this->FPS = this->get_parameter("config.fps").as_int();
        this->DEBUG = this->get_parameter("config.debug").as_bool();
        this->USE_LIDAR = this->get_parameter("config.use_lidar").as_bool();

        // Subscriber Initialization
        this->mode_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::mode_callback, this, std::placeholders::_1));
        this->gps_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/gps", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::gps_control_callback, this, std::placeholders::_1));
        this->compass_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/compass", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::compass_control_callback, this, std::placeholders::_1));
        this->vision_control_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/control/vision", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::vision_control_callback, this, std::placeholders::_1));
        this->lidar_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/lidar", rclcpp::SensorDataQoS(), std::bind(&UnderwaterNode::lidar_callback, this, std::placeholders::_1));
        this->snapshot_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/snapshot", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::snapshot_callback, this, std::placeholders::_1));
        this->mission_sub = this->create_subscription<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::mission_callback, this, std::placeholders::_1));
        this->reset_mission_sub = this->create_subscription<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS(), std::bind(&UnderwaterNode::reset_mission_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->underwater_image_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("/robot/image/underwater", rclcpp::ServicesQoS());
        this->underwater_mission_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/mission/underwater", rclcpp::ServicesQoS());
        this->mission_pub = this->create_publisher<std_msgs::msg::String>("/robot/mission", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / this->FPS), std::bind(&UnderwaterNode::runner, this));

        RCLCPP_INFO(this->get_logger(), "U-VIZ → node started");
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

    void snapshot_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->snapshot = msg->data;
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

    void publish_compressed_image(const cv::Mat& frame) {
        std::vector<uchar> buf;
        std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 90 };
        cv::imencode(".jpg", frame, buf, params);

        auto compressed_msg = sensor_msgs::msg::CompressedImage();
        compressed_msg.header.stamp = this->now();
        compressed_msg.format = "jpeg";
        compressed_msg.data = buf;

        // Multiple publish with threading to ensure delivery
        for (int i = 0; i < 3; i++) {
            this->underwater_image_pub->publish(compressed_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Processor method
    void processor(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(this->mutex);

        if (this->auto_mode) {
            if (!this->gps_control && !this->compass_control && this->vision_control) {
                if (this->mission == "Underwater Imaging") {
                    bool proximity_trigger = this->USE_LIDAR ? this->lidar : this->snapshot;

                    if (proximity_trigger && !this->is_underwater) {
                        this->is_underwater = true;
                        this->publish_compressed_image(frame);
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

            this->cam.set(cv::CAP_PROP_BUFFERSIZE, 1);
            this->cam.set(cv::CAP_PROP_FRAME_WIDTH, this->WIDTH);
            this->cam.set(cv::CAP_PROP_FRAME_HEIGHT, this->HEIGHT);
            this->cam.set(cv::CAP_PROP_FPS, this->FPS);

            camera_initialized = true;
            RCLCPP_INFO(this->get_logger(), "U-VIZ → Camera opened successfully: /dev/video%d", this->camera_device);
        }

        cv::Mat frame;
        bool success = this->cam.read(frame);

        if (success && !frame.empty()) {
            processor(frame);

            // Debug: Show underwater camera feed
            if (this->DEBUG) {
                cv::imshow("Underwater Camera", frame);
                cv::waitKey(1);
            }
        }

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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr snapshot_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_mission_sub;

    // Publisher Declaration
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr underwater_image_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr underwater_mission_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Variable Initialization
    int camera_device;
    int WIDTH;
    int HEIGHT;
    int FPS;
    bool DEBUG;
    bool USE_LIDAR;
    cv::VideoCapture cam;
    bool auto_mode = true;
    bool gps_control = false;
    bool compass_control = false;
    bool vision_control = false;
    bool lidar = false;
    bool snapshot = false;
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