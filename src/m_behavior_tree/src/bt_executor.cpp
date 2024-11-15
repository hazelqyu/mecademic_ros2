#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "m_behavior_tree/bt_nodes.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    // Register custom nodes
    RegisterNodes(factory);

    // Load the tree from the XML file
    auto tree = factory.createTreeFromFile("/home/andrek/ros2_ws/src/m_behavior_tree/config/my_behavior_tree.xml");

    // Execute the tree until it completes
    tree.tickWhileRunning();

    // Shutdown ROS when complete
    rclcpp::shutdown();
    return 0;
}
