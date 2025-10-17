#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mutex>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <serial/serial.h>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class RemoteNode : public rclcpp::Node {
public:
    RemoteNode()
        : Node("remote_node")
        , buffer("") {

        // Parameter Declaration
        this->declare_parameter<std::string>("arduino.port", "/dev/ttyACM1");
        this->declare_parameter<int>("arduino.baudrate", 9600);

        // Parameter Setup
        load_parameter();

        // Initialize waypoint file system
        this->waypoints_file_path = initialize_waypoints_file();

        // Serial Configuration
        if (!connect_to_arduino()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Arduino");
            rclcpp::shutdown();
            return;
        }

        // Subscriber Initialization
        this->global_pos_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", rclcpp::SensorDataQoS(), std::bind(&RemoteNode::global_pos_callback, this, std::placeholders::_1));

        // Publisher Initialization
        this->rc_override_pub = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", rclcpp::ServicesQoS());
        this->reset_mission_pub = this->create_publisher<std_msgs::msg::Bool>("/mission/reset", rclcpp::ServicesQoS());
        this->arming_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/arming", rclcpp::ServicesQoS());
        this->mode_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/mode", rclcpp::ServicesQoS());
        this->setmode_pub = this->create_publisher<std_msgs::msg::String>("/robot/setmode", rclcpp::ServicesQoS());

        this->trajectory_pub = this->create_publisher<std_msgs::msg::String>("/robot/trajectory", rclcpp::ServicesQoS());
        this->docking_pub = this->create_publisher<std_msgs::msg::String>("/detect/docking", rclcpp::ServicesQoS());
        this->left_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/left_buoy", rclcpp::ServicesQoS());
        this->right_buoy_pub = this->create_publisher<std_msgs::msg::String>("/detect/right_buoy", rclcpp::ServicesQoS());

        // Waypoint update publisher for Web GCS
        this->waypoint_update_pub = this->create_publisher<std_msgs::msg::String>("/robot/waypoints/update", rclcpp::ServicesQoS());

        // Rate Timer Initialization
        this->loop_timer = this->create_wall_timer(20ms, std::bind(&RemoteNode::read_and_publish, this));
        RCLCPP_INFO(this->get_logger(), "Remote node started");
    }

private:
    void load_parameter() {
        this->port = this->get_parameter("arduino.port").as_string();
        this->baudrate = this->get_parameter("arduino.baudrate").as_int();

        RCLCPP_INFO(get_logger(), "Parameters loaded - Port: %s, Baudrate: %d", this->port.c_str(), this->baudrate);
    }

    void param_callback() {
        load_parameter();
        RCLCPP_INFO(this->get_logger(), "REMOTE → Parameters Reloaded!");
    }

    void global_pos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->current_position = *msg;
    }

    bool connect_to_arduino() {
        try {
            this->ser.setPort(this->port);
            this->ser.setBaudrate(this->baudrate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            this->ser.setTimeout(timeout);
            this->ser.open();

            if (this->ser.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "REMOTE → Arduino connected: %s @ %d", this->port.c_str(), this->baudrate);
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "REMOTE → Serial port open failed");
                return false;
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "REMOTE → Serial error: %s", e.what());
            return false;
        }
    }

    std::string initialize_waypoints_file() {
        // Create waypoint directory in sensor package
        std::filesystem::path waypoints_dir = std::filesystem::path(__FILE__).parent_path() / "waypoint";
        try {
            if (!std::filesystem::exists(waypoints_dir)) {
                std::filesystem::create_directories(waypoints_dir);
                RCLCPP_INFO(this->get_logger(), "Created waypoints directory: %s", waypoints_dir.c_str());
            }
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create waypoints directory: %s", e.what());
            return "";
        }

        // Generate timestamped filename
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

        std::string filename = "waypoints_" + timestamp_stream.str() + ".txt";
        std::filesystem::path filepath = waypoints_dir / filename;

        // Initialize waypoint file with YAML structure
        try {
            std::ofstream file(filepath);
            if (file.is_open()) {
                file << "/gps_navigation:\n";
                file << "  ros__parameters: \n";
                file << "    navigation:\n";
                file << "      waypoints_X:\n";
                file.close();
                RCLCPP_INFO(this->get_logger(), "Initialized waypoints file: %s", filepath.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create waypoints file: %s", filepath.c_str());
                return "";
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error creating waypoints file: %s", e.what());
            return "";
        }

        return filepath.string();
    }

    void write_waypoint_to_file(const std::map<std::string, double>& waypoint) {
        try {
            std::ofstream file(this->waypoints_file_path, std::ios::app);
            if (file.is_open()) {
                file << "        waypoint_" << this->waypoint_counter << ":\n";
                file << "          latitude: " << std::fixed << std::setprecision(8) << waypoint.at("latitude") << "\n";
                file << "          longitude: " << std::fixed << std::setprecision(8) << waypoint.at("longitude") << "\n";
                file << "          altitude: " << std::fixed << std::setprecision(2) << waypoint.at("altitude") << "\n";
                file.close();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints file for writing: %s", this->waypoints_file_path.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error writing waypoint to file: %s", e.what());
        }
    }

    void publish_waypoint_update() {
        try {
            // Build JSON message with all waypoints
            std::stringstream json_msg;
            json_msg << "{\n";
            json_msg << "  \"session_id\": \"" << std::filesystem::path(this->waypoints_file_path).stem().string() << "\",\n";
            json_msg << "  \"timestamp\": \"" << get_current_timestamp() << "\",\n";
            json_msg << "  \"waypoints\": [\n";

            for (size_t i = 0; i < this->waypoints_list.size(); ++i) {
                const auto& wp = this->waypoints_list[i];
                json_msg << "    {\n";
                json_msg << "      \"id\": " << i << ",\n";
                json_msg << "      \"latitude\": " << std::fixed << std::setprecision(8) << wp.at("latitude") << ",\n";
                json_msg << "      \"longitude\": " << std::fixed << std::setprecision(8) << wp.at("longitude") << ",\n";
                json_msg << "      \"altitude\": " << std::fixed << std::setprecision(2) << wp.at("altitude") << "\n";
                json_msg << "    }";
                if (i < this->waypoints_list.size() - 1) {
                    json_msg << ",";
                }
                json_msg << "\n";
            }

            json_msg << "  ]\n";
            json_msg << "}";

            auto msg = std_msgs::msg::String();
            msg.data = json_msg.str();
            this->waypoint_update_pub->publish(msg);

            RCLCPP_DEBUG(this->get_logger(), "Published waypoint update to /robot/waypoints/update");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error publishing waypoint update: %s", e.what());
        }
    }

    std::string get_current_timestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream ss;
        ss << std::put_time(std::gmtime(&time_t_now), "%Y-%m-%dT%H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << 'Z';
        return ss.str();
    }

    bool is_valid_gps_position() {
        double lat = this->current_position.latitude;
        double lon = this->current_position.longitude;

        if (lat != 0.0 && lon != 0.0) {
            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
                return false;
            }
        }

        return true;
    }

    void waypoint_fetcher(int part) {
        std::lock_guard<std::mutex> lock(this->mutex);

        auto current_time = this->now();
        if (part <= 1500) {
            // Reset waypoint fetch timing when remote control is in low position
            this->waypoint_fetch_time = rclcpp::Time(0);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waypoint fetcher disabled (RC value: %d)", part);
        } else if (part > 1500) {
            if (!is_valid_gps_position()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Cannot fetch waypoint: Invalid GPS position (lat: %.8f, lon: %.8f)", this->current_position.latitude, this->current_position.longitude);
                return;
            }

            if (this->waypoint_fetch_time.nanoseconds() == 0 || (current_time - this->waypoint_fetch_time).seconds() >= 3) {

                std::map<std::string, double> new_waypoint;
                new_waypoint["latitude"] = this->current_position.latitude;
                new_waypoint["longitude"] = this->current_position.longitude;
                new_waypoint["altitude"] = 0.0;

                this->waypoints_list.push_back(new_waypoint);
                this->waypoint_fetch_time = current_time;

                if (!this->waypoints_file_path.empty()) {
                    write_waypoint_to_file(new_waypoint);
                }

                // Publish waypoint update to ROS topic for Web GCS
                publish_waypoint_update();

                // Log different message if GPS coordinates are zero
                if (this->current_position.latitude == 0.0 && this->current_position.longitude == 0.0) {
                    RCLCPP_INFO(this->get_logger(), "Fetched waypoint_%zu: (0.0, 0.0) - GPS not available", this->waypoint_counter);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Fetched waypoint_%zu: (%.8f, %.8f)", this->waypoint_counter, new_waypoint["latitude"], new_waypoint["longitude"]);
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Total Waypoints: %zu", this->waypoints_list.size());

                this->waypoint_counter++;
            } else {
                double time_remaining = 3.0 - (current_time - this->waypoint_fetch_time).seconds();
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waypoint fetch cooldown: %.1fs remaining", time_remaining);
            }
        }
    }

    void trajectory_switcher(int part) {
        auto msg_str = std_msgs::msg::String();

        if (part <= 1500 && this->prev_values["trajectory"] != "A") {
            msg_str.data = "A";
            this->trajectory_pub->publish(msg_str);

            msg_str.data = "red_buoy";
            this->left_buoy_pub->publish(msg_str);
            msg_str.data = "green_buoy";
            this->right_buoy_pub->publish(msg_str);
            msg_str.data = "red_dock";
            this->docking_pub->publish(msg_str);

            this->prev_values["trajectory"] = "A";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Attempting to set 'A' Trajectory...");

        } else if (part > 1500 && this->prev_values["trajectory"] != "B") {
            msg_str.data = "B";
            this->trajectory_pub->publish(msg_str);

            msg_str.data = "green_buoy";
            this->left_buoy_pub->publish(msg_str);
            msg_str.data = "red_buoy";
            this->right_buoy_pub->publish(msg_str);
            msg_str.data = "green_dock";
            this->docking_pub->publish(msg_str);

            this->prev_values["trajectory"] = "B";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Attempting to set 'B' Trajectory...");
        }
    }

    void mode_switcher(int part) {
        auto msg_str = std_msgs::msg::String();
        auto msg_bool = std_msgs::msg::Bool();

        if (part <= 1400 && this->prev_values["mode"] != "Manual") {
            msg_str.data = "MANUAL";
            this->setmode_pub->publish(msg_str);

            msg_bool.data = true;
            this->reset_mission_pub->publish(msg_bool);

            msg_bool.data = false;
            this->mode_pub->publish(msg_bool);

            this->remote_controlled = true;
            this->prev_values["mode"] = "Manual";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Attempting to set RC MANUAL mode...");

        } else if (1400 < part && part < 1600 && this->prev_values["mode"] != "Neutral") {
            msg_str.data = "MANUAL";
            this->setmode_pub->publish(msg_str);

            msg_bool.data = true;
            this->reset_mission_pub->publish(msg_bool);

            msg_bool.data = false;
            this->mode_pub->publish(msg_bool);

            this->remote_controlled = true;
            this->prev_values["mode"] = "Neutral";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Attempting to set RC NEUTRAL mode...");

        } else if (part >= 1600 && this->prev_values["mode"] != "Auto") {
            msg_str.data = "MANUAL";
            this->setmode_pub->publish(msg_str);

            msg_bool.data = false;
            this->reset_mission_pub->publish(msg_bool);

            msg_bool.data = true;
            this->mode_pub->publish(msg_bool);

            this->remote_controlled = false;
            this->prev_values["mode"] = "Auto";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Attempting to set RC AUTO mode...");
        }
    }

    void arming_switcher(int part) {
        auto msg_bool = std_msgs::msg::Bool();

        if (part <= 1500 && this->prev_values["arming"] != "Disarm") {
            msg_bool.data = false;
            this->arming_pub->publish(msg_bool);
            this->prev_values["arming"] = "Disarm";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Requesting DISARMING command...");

        } else if (part > 1500 && this->prev_values["arming"] != "Arm") {
            msg_bool.data = true;
            this->arming_pub->publish(msg_bool);
            this->prev_values["arming"] = "Arm";
            RCLCPP_INFO(this->get_logger(), "REMOTE → Requesting ARMING command...");
        }
    }

    void remote_control(int yaw, int pitch, int throttle, int lateral) {
        this->rc_override.channels.fill(65535);
        this->rc_override.channels[0] = yaw;
        this->rc_override.channels[1] = pitch;
        this->rc_override.channels[2] = throttle;
        this->rc_override.channels[3] = lateral;
        this->rc_override_pub->publish(this->rc_override);
    }

    void process_arduino_data(const std::string& line) {
        std::istringstream iss(line);
        std::vector<int> data_parts;
        int value;
        while (iss >> value) {
            data_parts.push_back(value);
        }

        if (data_parts.size() == 10) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "REMOTE → Received data: %s", line.c_str());

            try {
                int part_0 = data_parts[0]; // Yaw
                int part_1 = data_parts[1]; // Pitch
                int part_2 = data_parts[2]; // Throttle
                int part_3 = data_parts[3]; // Lateral
                int part_4 = data_parts[4]; // Waypoint Fetcher
                int part_5 = data_parts[5]; // Trajectory A - B
                int part_6 = data_parts[6]; // RC Auto - RC Neutral+Reset - RC Manual+Reset
                int part_7 = data_parts[7]; // Arming - Disarming

                waypoint_fetcher(part_4);
                trajectory_switcher(part_5);
                mode_switcher(part_6);
                arming_switcher(part_7);

                if (this->remote_controlled) {
                    if (this->prev_values["mode"] == "Neutral") {
                        remote_control(1500, 1500, 1500, 1500);
                    } else if (this->prev_values["mode"] == "Manual") {
                        remote_control(part_0, part_1, part_2, part_3);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "REMOTE → Error processing Arduino data: %s", e.what());
            }
        }
    }

    void read_and_publish() {
        try {
            if (this->ser.available() > 0) {
                this->buffer += this->ser.read(this->ser.available());
                if (this->buffer.find('\n') != std::string::npos) {
                    std::string line = this->buffer.substr(0, this->buffer.find('\n'));
                    this->buffer = this->buffer.substr(this->buffer.find('\n') + 1);
                    process_arduino_data(line);
                }
            }
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "REMOTE → Error reading from serial port: %s", e.what());
        }
    }

    // Publisher Declaration
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_mission_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arming_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setmode_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_buoy_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_buoy_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_update_pub;

    // Subscriber Declaration
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub;

    // Rate Timer Declaration
    rclcpp::TimerBase::SharedPtr loop_timer;
    rclcpp::TimerBase::SharedPtr param_timer;

    // Mutex Declaration
    std::mutex mutex;

    // Serial Declaration
    serial::Serial ser;
    std::string port;
    int baudrate;
    std::string buffer;

    // Variable Declarations
    sensor_msgs::msg::NavSatFix current_position;
    mavros_msgs::msg::OverrideRCIn rc_override;
    std::vector<std::map<std::string, double>> waypoints_list;
    std::string waypoints_file_path;
    rclcpp::Time waypoint_fetch_time;

    // Variable Initialization
    std::map<std::string, std::string> prev_values { { "arming", "" }, { "mode", "" }, { "trajectory", "" } };
    size_t waypoint_counter = 0;
    bool remote_controlled = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
