#include <rclcpp/rclcpp.hpp>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode()
        : Node("vision_planner_node") {

        RCLCPP_INFO(get_logger(), "GPS Vision Planner node started");
        // TODO: Implement vision planner functionality
    }

private:
    // TODO: Add vision planner implementation
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}