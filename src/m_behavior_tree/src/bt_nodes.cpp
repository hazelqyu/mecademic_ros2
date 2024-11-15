#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

// Custom Action Node: TrackFace
class TrackFace : public BT::SyncActionNode {
public:
    TrackFace(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("TrackFace"), "Tracking face...");
        return BT::NodeStatus::SUCCESS;
    }
};

// Custom Action Node: Idle
class Idle : public BT::SyncActionNode {
public:
    Idle(const std::string& name) : BT::SyncActionNode(name, {}) {}

    BT::NodeStatus tick() override {
        RCLCPP_INFO(rclcpp::get_logger("Idle"), "Idling...");
        return BT::NodeStatus::SUCCESS;
    }
};

void RegisterNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<TrackFace>("TrackFace");
    factory.registerNodeType<Idle>("Idle");
}