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
    // if (!callStateChangeService(node_)) {
    //     RCLCPP_ERROR(rclcpp::get_logger("TrackFace"), "Failed to call state change service.");
    //     return false;  // Return false if the service call fails
    // }
    msg.data = true;  // Always publish a "start tracking" signal
    RCLCPP_INFO(rclcpp::get_logger("TrackFace"), "Publishing tracking signal.");
    return true;  // Return true to indicate successful message setup
}


Idle::Idle(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config){

    ros_node_ = rclcpp::Node::make_shared("idle_node");
    publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/start_idling", 10);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle node initialized.");
}

// Ports definition
BT::PortsList Idle::providedPorts(){
    return {};
}

BT::NodeStatus Idle::onStart(){
    while (publisher_->get_subscription_count() == 0) {
        RCLCPP_INFO(ros_node_->get_logger(), "Waiting for subscriber to /start_idling...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!callStateChangeService(node_)) {
        RCLCPP_ERROR(rclcpp::get_logger("Idle"), "Failed to call state change service.");
        return BT::NodeStatus::FAILURE;
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle node started. Published True to /start_idling.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Idle::onRunning(){
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    publisher_->publish(msg);
    RCLCPP_INFO(ros_node_->get_logger(), "Idle node running.");
    return BT::NodeStatus::RUNNING;
}

void Idle::onHalted(){
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle node halted. Published False to /start_idling.");
}




// IsDetectedCondition Node
IsDetectedCondition::IsDetectedCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(false) {}

BT::NodeStatus IsDetectedCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsDetectedCondition"), 
        "Face detected: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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

