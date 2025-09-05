#include <rclcpp/rclcpp.hpp>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode()
        : Node("planner_node") {

        RCLCPP_INFO(this->get_logger(), "GPS RTK Planner node started");
        // TODO: Implement GPS RTK planner functionality
    }

private:
    // TODO: Add GPS RTK planner implementation
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}