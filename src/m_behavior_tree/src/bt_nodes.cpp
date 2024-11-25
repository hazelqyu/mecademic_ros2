#include "m_behavior_tree/bt_nodes.h"  // Ensure this is the correct path
#include "custom_interfaces/srv/clear_motion.hpp"
#include <chrono>

bool callStateChangeService(rclcpp::Node::SharedPtr node);

TrackFace::TrackFace(
    const std::string& name,
    const BT::NodeConfig& config,
    const BT::RosNodeParams& params
) : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, config, params) {
    node_ = rclcpp::Node::make_shared("track_face_btree_node");
}

BT::PortsList TrackFace::providedPorts() {
    // Default ports for topic name; add more if needed
    return BT::RosTopicPubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

bool TrackFace::setMessage(std_msgs::msg::Bool& msg) {
    if (!callStateChangeService(node_)) {
        RCLCPP_ERROR(rclcpp::get_logger("TrackFace"), "Failed to call state change service.");
        return false;  // Return false if the service call fails
    }
    msg.data = true;  // Always publish a "start tracking" signal
    RCLCPP_INFO(rclcpp::get_logger("TrackFace"), "Publishing tracking signal.");
    return true;  // Return true to indicate successful message setup
}

Idle::Idle(
    const std::string& name,
    const BT::NodeConfig& config,
    const BT::RosNodeParams& params
) : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, config, params) {
    node_ = rclcpp::Node::make_shared("idle_btree_node");
}

BT::PortsList Idle::providedPorts() {
    // Default ports for topic name; add more if needed
    return BT::RosTopicPubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

bool Idle::setMessage(std_msgs::msg::Bool& msg) {
    if (!callStateChangeService(node_)) {
        RCLCPP_ERROR(rclcpp::get_logger("TrackFace"), "Failed to call state change service.");
        return false;  // Return false if the service call fails
    }
    msg.data = true;  // Always publish a "start tracking" signal
    RCLCPP_INFO(rclcpp::get_logger("Idle"), "Publishing Idling signal.");
    return true;  // Return true to indicate successful message setup
}

// Idle::Idle(const std::string& name) : BT::SyncActionNode(name, {}) {}

// BT::NodeStatus Idle::tick() {
//     RCLCPP_INFO(rclcpp::get_logger("Idle"), "Idling...");
//     return BT::NodeStatus::SUCCESS;
// }

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

bool callStateChangeService(rclcpp::Node::SharedPtr node) {
    // Create a client for the service
    auto client = node->create_client<custom_interfaces::srv::ClearMotion>("/state_change");
    
    // Wait for the service to be available
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "State change service not available.");
        return false;
    }

    // Create the request
    auto request = std::make_shared<custom_interfaces::srv::ClearMotion::Request>();

    // Call the service
    auto future = client->async_send_request(request);

    // Wait for the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        return future.get()->success;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call state change service.");
        return false;
    }
}

