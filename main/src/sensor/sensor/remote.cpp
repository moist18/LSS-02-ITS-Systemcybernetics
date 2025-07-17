#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

class RemoteNode : public rclcpp::Node {
public:
    RemoteNode()
        : Node("remote_node")
        , buffer_("")
        , rate_(50) {
        // Declare and get parameters
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        baudrate_ = this->declare_parameter<int>("baudrate", 9600);

        // Initialize serial connection
        if (!connectToArduino()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Arduino");
            rclcpp::shutdown();
            return;
        }

        // Publishers
        rc_override_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        reset_mission_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mission/reset", 10);
        arming_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/arming", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot/mode", 10);
        setmode_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/setmode", 10);

        trajectory_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/trajectory", 10);
        docking_pub_ = this->create_publisher<std_msgs::msg::String>("/detect/docking", 10);
        left_buoy_pub_ = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", 10);
        right_buoy_pub_ = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", 10);

        // Subscriber
        global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", 10,
            std::bind(&RemoteNode::globalPosCallback, this, std::placeholders::_1));

        // Initialize waypoint file
        // initializeWaypointsFile();

        // Timer to periodically read from Arduino
        timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&RemoteNode::readAndPublish, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_mission_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setmode_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_buoy_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_buoy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;

    serial::Serial ser_;
    std::string port_;
    int baudrate_;
    std::string buffer_;
    double rate_;

    sensor_msgs::msg::NavSatFix current_position_;
    mavros_msgs::msg::OverrideRCIn rc_override_;

    std::vector<std::string> waypoints_;
    std::string waypoints_file_path_;
    size_t waypoint_counter_ = 0;

    // Remote control state variables
    bool remote_controlled_ = true;
    rclcpp::Time waypoint_fetch_time_;
    std::map<std::string, std::string> prev_values_ { { "arming", "" }, { "mode", "" }, { "trajectory", "" } };

    bool connectToArduino() {
        try {
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(timeout);
            ser_.open();

            if (ser_.isOpen()) {
                RCLCPP_INFO(
                    this->get_logger(), "Connected to Arduino on port %s at %d baudrate.", port_.c_str(), baudrate_);
                return true;
            } else {
                return false;
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
            return false;
        }
    }

    void globalPosCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { current_position_ = *msg; }

    void readAndPublish() {
        try {
            if (ser_.available()) {
                buffer_ += ser_.read(ser_.available());
                size_t pos = buffer_.find('\n');
                while (pos != std::string::npos) {
                    std::string line = buffer_.substr(0, pos);
                    buffer_ = buffer_.substr(pos + 1);
                    processArduinoData(line);
                    pos = buffer_.find('\n');
                }
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read exception: %s", e.what());
        }
    }

    void processArduinoData(const std::string& line) {
        std::istringstream iss(line);
        std::vector<int> data_parts;
        int value;
        while (iss >> value) {
            data_parts.push_back(value);
        }

        if (data_parts.size() == 10) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received data: %s", line.c_str());

            try {
                int part_0 = data_parts[0]; // Yaw
                int part_1 = data_parts[1]; // Pitch
                int part_2 = data_parts[2]; // Throttle
                int part_3 = data_parts[3]; // Lateral
                int part_4 = data_parts[4]; // Waypoint Fetcher
                int part_5 = data_parts[5]; // Trajectory A - B
                int part_6 = data_parts[6]; // RC Auto - RC Neutral+Reset - RC Manual+Reset
                int part_7 = data_parts[7]; // Arming - Disarming

                waypointFetcher(part_4);
                trajectorySwitcher(part_5);
                modeSwitcher(part_6);
                armingSwitcher(part_7);

                if (remote_controlled_) {
                    if (prev_values_["mode"] == "Neutral") {
                        remoteControl(1500, 1500, 1500, 1500);
                    } else if (prev_values_["mode"] == "Manual") {
                        remoteControl(part_0, part_1, part_2, part_3);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error processing Arduino data: %s", e.what());
            }
        }
    }

    void waypointFetcher(int part) {
        auto current_time = this->now();
        if (part <= 1500) {
            waypoint_fetch_time_ = rclcpp::Time(0);
        } else if (part > 1500 && current_position_.header.stamp.sec != 0) {
            if (waypoint_fetch_time_.nanoseconds() == 0 || (current_time - waypoint_fetch_time_).seconds() >= 3) {

                waypoint_fetch_time_ = current_time;

                std::ostringstream waypoint_info;
                waypoint_info << "Waypoint " << waypoint_counter_ << ": " << current_position_.latitude << ", "
                              << current_position_.longitude;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000, "Fetched %s", waypoint_info.str().c_str());
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000, "Total Waypoints: %zu", waypoint_counter_ + 1);

                waypoint_counter_++;
            }
        }
    }

    void trajectorySwitcher(int part) {
        auto msg_str = std_msgs::msg::String();

        if (part <= 1500 && prev_values_["trajectory"] != "A") {
            msg_str.data = "A";
            trajectory_pub_->publish(msg_str);

            msg_str.data = "red_buoy";
            left_buoy_pub_->publish(msg_str);
            msg_str.data = "green_buoy";
            right_buoy_pub_->publish(msg_str);
            msg_str.data = "red_dock";
            docking_pub_->publish(msg_str);

            prev_values_["trajectory"] = "A";
            RCLCPP_INFO(this->get_logger(), "Attempting to set 'A' Trajectory...");

        } else if (part > 1500 && prev_values_["trajectory"] != "B") {
            msg_str.data = "B";
            trajectory_pub_->publish(msg_str);

            msg_str.data = "green_buoy";
            left_buoy_pub_->publish(msg_str);
            msg_str.data = "red_buoy";
            right_buoy_pub_->publish(msg_str);
            msg_str.data = "green_dock";
            docking_pub_->publish(msg_str);

            prev_values_["trajectory"] = "B";
            RCLCPP_INFO(this->get_logger(), "Attempting to set 'B' Trajectory...");
        }
    }

    void modeSwitcher(int part) {
        auto msg_str = std_msgs::msg::String();
        auto msg_bool = std_msgs::msg::Bool();

        if (part <= 1400 && prev_values_["mode"] != "Manual") {
            msg_str.data = "MANUAL";
            setmode_pub_->publish(msg_str);

            msg_bool.data = true;
            reset_mission_pub_->publish(msg_bool);

            msg_bool.data = false;
            mode_pub_->publish(msg_bool);

            remote_controlled_ = true;
            prev_values_["mode"] = "Manual";
            RCLCPP_INFO(this->get_logger(), "Attempting to set RC MANUAL mode...");

        } else if (1400 < part && part < 1600 && prev_values_["mode"] != "Neutral") {
            msg_str.data = "MANUAL";
            setmode_pub_->publish(msg_str);

            msg_bool.data = true;
            reset_mission_pub_->publish(msg_bool);

            msg_bool.data = false;
            mode_pub_->publish(msg_bool);

            remote_controlled_ = true;
            prev_values_["mode"] = "Neutral";
            RCLCPP_INFO(this->get_logger(), "Attempting to set RC NEUTRAL mode...");

        } else if (part >= 1600 && prev_values_["mode"] != "Auto") {
            msg_str.data = "GUIDED";
            setmode_pub_->publish(msg_str);

            msg_bool.data = false;
            reset_mission_pub_->publish(msg_bool);

            msg_bool.data = true;
            mode_pub_->publish(msg_bool);

            remote_controlled_ = false;
            prev_values_["mode"] = "Auto";
            RCLCPP_INFO(this->get_logger(), "Attempting to set RC AUTO mode...");
        }
    }

    void armingSwitcher(int part) {
        auto msg_bool = std_msgs::msg::Bool();

        if (part <= 1500 && prev_values_["arming"] != "Disarm") {
            msg_bool.data = false;
            arming_pub_->publish(msg_bool);
            prev_values_["arming"] = "Disarm";
            RCLCPP_INFO(this->get_logger(), "Requesting DISARMING command...");

        } else if (part > 1500 && prev_values_["arming"] != "Arm") {
            msg_bool.data = true;
            arming_pub_->publish(msg_bool);
            prev_values_["arming"] = "Arm";
            RCLCPP_INFO(this->get_logger(), "Requesting ARMING command...");
        }
    }

    void remoteControl(int yaw, int pitch, int throttle, int lateral) {
        rc_override_.channels.fill(65535);
        rc_override_.channels[0] = yaw;
        rc_override_.channels[1] = pitch;
        rc_override_.channels[2] = throttle;
        rc_override_.channels[3] = lateral;
        rc_override_pub_->publish(rc_override_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
