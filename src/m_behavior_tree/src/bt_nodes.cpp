#include "m_behavior_tree/bt_nodes.h"  // Ensure this is the correct path

TrackFace::TrackFace(
    const std::string& name,
    const BT::NodeConfig& config,
    const BT::RosNodeParams& params
) : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, config, params) {}

BT::PortsList TrackFace::providedPorts() {
    // Default ports for topic name; add more if needed
    return BT::RosTopicPubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

bool TrackFace::setMessage(std_msgs::msg::Bool& msg) {
    // This node only runs when the parent tree determines it should start tracking
    msg.data = true;  // Always publish a "start tracking" signal
    RCLCPP_INFO(rclcpp::get_logger("TrackFace"), "Publishing start tracking signal.");
    return true;  // Return true to indicate successful message setup
}

// Idle Action Node
Idle::Idle(const std::string& name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus Idle::tick() {
    RCLCPP_INFO(rclcpp::get_logger("Idle"), "Idling...");
    return BT::NodeStatus::SUCCESS;
}

// IsDetectedCondition Node
IsDetectedCondition::IsDetectedCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params) {}

BT::NodeStatus IsDetectedCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {
    RCLCPP_INFO(
        rclcpp::get_logger("IsDetectedCondition"), 
        "Face detected: %s", 
        last_msg ? (last_msg->data ? "True" : "False") : "null"
    );
    if (last_msg) {
        // RCLCPP_INFO(rclcpp::get_logger("IsDetectedCondition"), "Face detected: %s", last_msg->data ? "True" : "False");
        return last_msg->data ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::FAILURE;
}

BT::PortsList IsDetectedCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

