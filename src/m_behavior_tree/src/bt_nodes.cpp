#include "m_behavior_tree/bt_nodes.h"
#include "m_behavior_tree/service_helpers.h" 
#include "custom_interfaces/srv/clear_motion.hpp"
#include "custom_interfaces/srv/execute_motion.hpp"
#include <chrono>

bool callStateChangeService(rclcpp::Node::SharedPtr node);

//TrackFace Node

TrackFace::TrackFace(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config){
    ros_node_ = rclcpp::Node::make_shared("track_face_btree_node");
    publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/start_tracking", 10);

    RCLCPP_INFO(ros_node_->get_logger(), "TrackFace tree node initialized.");
}

BT::PortsList TrackFace::providedPorts(){
    return {};
}

BT::NodeStatus TrackFace::onStart(){
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in TrackFace node.");
        return BT::NodeStatus::FAILURE;
    }
    while (publisher_->get_subscription_count() == 0) {
        RCLCPP_INFO(ros_node_->get_logger(), "Waiting for subscriber to /start_tracking...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "TrackFace node started. Published True to /start_tracking.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TrackFace::onRunning(){
    RCLCPP_INFO(ros_node_->get_logger(), "TrackFace node running.");
    return BT::NodeStatus::RUNNING;
}

void TrackFace::onHalted(){
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "TrackFace node halted. Published False to /start_tracking.");
}


//Idle Node


Idle::Idle(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config){

    ros_node_ = rclcpp::Node::make_shared("idle_tree_node");
    publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/start_idling", 10);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle tree node initialized.");
}

// Ports definition
BT::PortsList Idle::providedPorts(){
    return {};
}

BT::NodeStatus Idle::onStart(){
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Idle node.");
        return BT::NodeStatus::FAILURE;
    }
    while (publisher_->get_subscription_count() == 0) {
        RCLCPP_INFO(ros_node_->get_logger(), "Waiting for subscriber to /start_idling...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle node started. Published True to /start_idling.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Idle::onRunning(){
    RCLCPP_INFO(ros_node_->get_logger(), "Idle node running.");
    return BT::NodeStatus::RUNNING;
}

void Idle::onHalted(){
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Idle node halted. Published False to /start_idling.");
}


//Yawn Node

Yawn::Yawn(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      ros_node_(rclcpp::Node::make_shared("yawn_tree_node")),
      motion_client_(ros_node_, "/execute_motion"),
      motion_started_(false) {
    RCLCPP_INFO(ros_node_->get_logger(), "Yawn tree node initialized.");
}

// Define the ports (empty in this case)
BT::PortsList Yawn::providedPorts() {
    return {};
}

BT::NodeStatus Yawn::onStart() {
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Yawn...");
        motion_client_.sendRequest("yawn");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Yawn::onRunning() {
    // Check if the service is complete
    if (motion_client_.isServiceComplete()) {
        RCLCPP_INFO(ros_node_->get_logger(), "Yawn motion completed successfully.");
        this->halt();
        return BT::NodeStatus::SUCCESS; // Mark the node as complete
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Yawn motion still running...");
    return BT::NodeStatus::RUNNING; // Keep running while the service is not complete
}

void Yawn::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(), "Yawn node halted.");
    motion_started_ = false; // Reset the motion_started_ flag
}


//IsDetectedCondition Node


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
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsDetectedCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

//IsBoredConditionNode

IsBoredCondition::IsBoredCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(true) {}

BT::NodeStatus IsBoredCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsBoredCondition"), 
        "Face detected: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBoredCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

bool callStateChangeService(rclcpp::Node::SharedPtr node) {
    
    auto client = node->create_client<custom_interfaces::srv::ClearMotion>("/state_change");
    
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if(!rclcpp::ok()){
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }

    auto request = std::make_shared<custom_interfaces::srv::ClearMotion::Request>();

    auto future = client->async_send_request(request);

    // Wait for the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        return future.get()->success;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call state change service.");
        return false;
    }
}
