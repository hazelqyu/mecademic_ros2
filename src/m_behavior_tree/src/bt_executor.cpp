#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "m_behavior_tree/bt_nodes.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("bt_executor_node");

    BT::BehaviorTreeFactory factory;

    // Set up ROS node parameters
    BT::RosNodeParams params;
    params.nh = nh;
    params.default_port_value = "is_detected";  // Default topic name

    // Register the IsDetectedCondition node with parameters
    factory.registerNodeType<IsDetectedCondition>("IsDetectedCondition", params);
    factory.registerNodeType<TrackFace>("TrackFace");
    factory.registerNodeType<Idle>("Idle");

    // Register custom nodes
    //RegisterNodes(factory);

    // Load the tree from the XML file
    auto tree = factory.createTreeFromFile("/home/andrek/ros2_ws/src/m_behavior_tree/config/my_behavior_tree.xml");

    // Execute the tree in a loop
    while (rclcpp::ok()) {
        tree.tickWhileRunning();
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // Adjust interval as needed
    }

    // rclcpp::shutdown();
    return 0;
}
