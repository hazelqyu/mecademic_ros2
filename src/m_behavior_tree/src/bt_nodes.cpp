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

// Asleep Node
Asleep::Asleep(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config){

    ros_node_ = rclcpp::Node::make_shared("asleep_tree_node");
    publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/start_sleeping", 10);

    RCLCPP_INFO(ros_node_->get_logger(), "Asleep tree node initialized.");
}

// Ports definition
BT::PortsList Asleep::providedPorts(){
    return {};
}

BT::NodeStatus Asleep::onStart(){
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Asleep node.");
        return BT::NodeStatus::FAILURE;
    }
    while (publisher_->get_subscription_count() == 0) {
        RCLCPP_INFO(ros_node_->get_logger(), "Waiting for subscriber to /start_sleeping...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Asleep node started. Published True to /start_sleeping.");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Asleep::onRunning(){
    RCLCPP_INFO(ros_node_->get_logger(), "Asleep node running.");
    return BT::NodeStatus::RUNNING;
}

void Asleep::onHalted(){
    auto msg = std_msgs::msg::Bool();
    msg.data = false;
    publisher_->publish(msg);

    RCLCPP_INFO(ros_node_->get_logger(), "Asleep node halted. Published False to /start_sleeping.");
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

//Dance Node
Dance::Dance(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      ros_node_(rclcpp::Node::make_shared("dance_tree_node")),
      motion_client_(ros_node_, "/execute_motion"),
      motion_started_(false) {
    RCLCPP_INFO(ros_node_->get_logger(), "Dance tree node initialized.");
}

// Define the ports (empty in this case)
BT::PortsList Dance::providedPorts() {
    return {};
}

BT::NodeStatus Dance::onStart() {
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Dance node.");
        return BT::NodeStatus::FAILURE;
    }
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Dance...");
        motion_client_.sendRequest("dance");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dance::onRunning() {
    ExecutionTimeTracker::getInstance().updateLastExecutionTime("Dance");
    // Check if the service is complete
    if (motion_client_.isServiceComplete()) {
        RCLCPP_INFO(ros_node_->get_logger(), "Dance motion completed successfully.");
        this->halt();
        return BT::NodeStatus::SUCCESS; // Mark the node as complete
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Dance motion still running...");
    return BT::NodeStatus::RUNNING; // Keep running while the service is not complete
}

void Dance::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(), "Dance node halted.");
    motion_started_ = false; // Reset the motion_started_ flag
}

//Dash Node

Dash::Dash(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      ros_node_(rclcpp::Node::make_shared("dash_tree_node")),
      motion_client_(ros_node_, "/execute_motion"),
      motion_started_(false) {
    RCLCPP_INFO(ros_node_->get_logger(), "Dash tree node initialized.");
}

// Define the ports (empty in this case)
BT::PortsList Dash::providedPorts() {
    return {};
}

BT::NodeStatus Dash::onStart() {
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Dash node.");
        return BT::NodeStatus::FAILURE;
    }
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Dash...");
        motion_client_.sendRequest("dash");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dash::onRunning() {
    ExecutionTimeTracker::getInstance().updateLastExecutionTime("Dash");
    // Check if the service is complete
    if (motion_client_.isServiceComplete()) {
        RCLCPP_INFO(ros_node_->get_logger(), "Dash motion completed successfully.");
        this->halt();
        return BT::NodeStatus::SUCCESS; // Mark the node as complete
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Dash motion still running...");
    return BT::NodeStatus::RUNNING; // Keep running while the service is not complete
}

void Dash::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(), "Dash node halted.");
    motion_started_ = false; // Reset the motion_started_ flag
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
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Yawn node.");
        return BT::NodeStatus::FAILURE;
    }
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Yawn...");
        motion_client_.sendRequest("yawn");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Yawn::onRunning() {
    ExecutionTimeTracker::getInstance().updateLastExecutionTime("Yawn");
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

// Alert Node

Alert::Alert(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      ros_node_(rclcpp::Node::make_shared("alert_tree_node")),
      motion_client_(ros_node_, "/execute_motion"),
      motion_started_(false) {
    RCLCPP_INFO(ros_node_->get_logger(), "Alert tree node initialized.");
}

// Define the ports (empty in this case)
BT::PortsList Alert::providedPorts() {
    return {};
}

BT::NodeStatus Alert::onStart() {
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Alert node.");
        return BT::NodeStatus::FAILURE;
    }
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Alert...");
        motion_client_.sendRequest("alert");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Alert::onRunning() {
    ExecutionTimeTracker::getInstance().updateLastExecutionTime("Alert");
    // Check if the service is complete
    if (motion_client_.isServiceComplete()) {
        RCLCPP_INFO(ros_node_->get_logger(), "Alert motion completed successfully.");
        this->halt();
        return BT::NodeStatus::SUCCESS; // Mark the node as complete
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Alert motion still running...");
    return BT::NodeStatus::RUNNING; // Keep running while the service is not complete
}

void Alert::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(), "Alert node halted.");
    motion_started_ = false; // Reset the motion_started_ flag
}

// Chomp Node

Chomp::Chomp(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config),
      ros_node_(rclcpp::Node::make_shared("chomp_tree_node")),
      motion_client_(ros_node_, "/execute_motion"),
      motion_started_(false) {
    RCLCPP_INFO(ros_node_->get_logger(), "Chomp tree node initialized.");
}

// Define the ports (empty in this case)
BT::PortsList Chomp::providedPorts() {
    return {};
}

BT::NodeStatus Chomp::onStart() {
    if (!callStateChangeService(ros_node_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to call ClearMotion service in Chomp node.");
        return BT::NodeStatus::FAILURE;
    }
    // Start the motion service if it hasn't been started yet
    if (!motion_started_) {
        RCLCPP_INFO(ros_node_->get_logger(), "Calling ExecuteMotion service for Chomp...");
        motion_client_.sendRequest("chomp");  // Send request to the service
        motion_started_ = true;             // Mark motion as started
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Chomp::onRunning() {
    ExecutionTimeTracker::getInstance().updateLastExecutionTime("Chomp");
    // Check if the service is complete
    if (motion_client_.isServiceComplete()) {
        RCLCPP_INFO(ros_node_->get_logger(), "Chomp motion completed successfully.");
        this->halt();
        return BT::NodeStatus::SUCCESS; // Mark the node as complete
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Chomp motion still running...");
    return BT::NodeStatus::RUNNING; // Keep running while the service is not complete
}

void Chomp::onHalted() {
    RCLCPP_INFO(ros_node_->get_logger(), "Chomp node halted.");
    motion_started_ = false; // Reset the motion_started_ flag
}

// IsScanningCondition Node
IsScanningCondition::IsScanningCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(false) {}

BT::NodeStatus IsScanningCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsScanningCondition"), 
        "Plant Scanning: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsScanningCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
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
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(false) {}

BT::NodeStatus IsBoredCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsBoredCondition"), 
        "IsBored: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsBoredCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

//IsAlertConditionNode

IsAlertCondition::IsAlertCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(false) {}

BT::NodeStatus IsAlertCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsAlertCondition"), 
        "IsAlert: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAlertCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}


// IsTooCloseConditionNode

IsTooCloseCondition::IsTooCloseCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, config, params),last_msg_value_(false) {}

BT::NodeStatus IsTooCloseCondition::onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsTooCloseCondition"), 
        "IsTooClose: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        last_msg_value_ ? "True" : "False"
    );
    return last_msg_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsTooCloseCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::Bool>::providedBasicPorts({});
}

// IsHappyConditionNode

IsHappyCondition::IsHappyCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::String>(name, config, params),last_msg_value_("") {}

BT::NodeStatus IsHappyCondition::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsHappyCondition"), 
        "IsHappy: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        (last_msg_value_ == "happy") ? "True" : "False"
    );
    return (last_msg_value_ == "happy") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsHappyCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::String>::providedBasicPorts({});
}

// IsAngryConditionNode

IsAngryCondition::IsAngryCondition(
    const std::string& name, 
    const BT::NodeConfig& config, 
    const BT::RosNodeParams& params
) : BT::RosTopicSubNode<std_msgs::msg::String>(name, config, params),last_msg_value_("") {}

BT::NodeStatus IsAngryCondition::onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) {

    if (last_msg) {
        last_msg_value_ = last_msg->data;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("IsAngryCondition"), 
        "IsAngry: %s", 
        // last_msg ? (last_msg->data ? "True" : "False") : "null"
        (last_msg_value_ == "angry") ? "True" : "False"
    );
    // return (last_msg_value_ == "angry") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    return ((last_msg_value_ == "angry") || 
            (last_msg_value_ == "surprise")) ? 
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAngryCondition::providedPorts() {
    return BT::RosTopicSubNode<std_msgs::msg::String>::providedBasicPorts({});
}

// ExecutionCheckNode

ExecutionCheck::ExecutionCheck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus ExecutionCheck::tick() {
    std::string node_name;
    int threshold;
    if (!getInput("node_name", node_name) || !getInput("threshold", threshold)) {
        throw BT::RuntimeError("Missing required input ports [node_name] or [threshold]");
    }

    auto& tracker = ExecutionTimeTracker::getInstance();
    auto last_execution_time = tracker.getLastExecutionTime(node_name);
    auto now = std::chrono::steady_clock::now();

    if (last_execution_time == std::chrono::steady_clock::time_point::min()) {
        // No execution time recorded; assume timeout has passed
        return BT::NodeStatus::SUCCESS;
    }

    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - last_execution_time).count();
    if (elapsed_time > threshold) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

//GlobalExecutionCheck
GlobalExecutionCheck::GlobalExecutionCheck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus GlobalExecutionCheck::tick() {
    int threshold;
    if (!getInput("threshold", threshold)) {
        throw BT::RuntimeError("Missing required input port [threshold]");
    }

    auto& tracker = ExecutionTimeTracker::getInstance();
    auto global_last_execution_time = tracker.getGlobalLastExecutionTime();
    auto now = std::chrono::steady_clock::now();

    if (global_last_execution_time == std::chrono::steady_clock::time_point::min()) {
        // No execution time recorded globally; assume timeout has passed
        return BT::NodeStatus::SUCCESS;
    }

    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - global_last_execution_time).count();
    if (elapsed_time > threshold) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
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
