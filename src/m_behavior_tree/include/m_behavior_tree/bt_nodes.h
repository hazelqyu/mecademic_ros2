#ifndef MY_BT_NODES_H
#define MY_BT_NODES_H

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "m_behavior_tree/service_helpers.h"
#include "m_behavior_tree/execution_time_tracker.h"


// Custom Action Node: TrackFace
// class TrackFace : public BT::RosTopicPubNode<std_msgs::msg::Bool> {
// public:
//     TrackFace(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
//     static BT::PortsList providedPorts();
// protected:
//     bool setMessage(std_msgs::msg::Bool& msg) override;
// };

class TrackFace : public BT::StatefulActionNode{
public:
    TrackFace(const std::string &name, const BT::NodeConfig &config);

    //StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

class Idle : public BT::StatefulActionNode{
public:
    Idle(const std::string &name, const BT::NodeConfig &config);

    //StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

class Asleep : public BT::StatefulActionNode{
public:
    Asleep(const std::string &name, const BT::NodeConfig &config);

    //StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};



class Yawn : public BT::StatefulActionNode {
public:
    Yawn(const std::string &name, const BT::NodeConfig &config);

    // StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    MotionServiceClient motion_client_;
    bool motion_started_;
};


class Alert : public BT::StatefulActionNode {
public:
    Alert(const std::string &name, const BT::NodeConfig &config);

    // StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    MotionServiceClient motion_client_;
    bool motion_started_;
};

class Chomp : public BT::StatefulActionNode {
public:
    Chomp(const std::string &name, const BT::NodeConfig &config);

    // StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    MotionServiceClient motion_client_;
    bool motion_started_;
};

class Dance : public BT::StatefulActionNode {
public:
    Dance(const std::string &name, const BT::NodeConfig &config);

    // StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    MotionServiceClient motion_client_;
    bool motion_started_;
};

class Dash : public BT::StatefulActionNode {
public:
    Dash(const std::string &name, const BT::NodeConfig &config);

    // StatefulActionNode methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Required to define ports
    static BT::PortsList providedPorts();

private:
    rclcpp::Node::SharedPtr ros_node_;
    MotionServiceClient motion_client_;
    bool motion_started_;
};


class IsScanningCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsScanningCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    bool last_msg_value_;
};


class IsDetectedCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsDetectedCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    bool last_msg_value_;
};

class IsBoredCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsBoredCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    bool last_msg_value_;
};

class IsAlertCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsAlertCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    bool last_msg_value_;
};

class IsTooCloseCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsTooCloseCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    bool last_msg_value_;
};


class IsHappyCondition : public BT::RosTopicSubNode<std_msgs::msg::String> {
public:
    IsHappyCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    std::string last_msg_value_;
};

class IsAngryCondition : public BT::RosTopicSubNode<std_msgs::msg::String> {
public:
    IsAngryCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::String>& last_msg) override;
    static BT::PortsList providedPorts();

private:
    // Member variable to store the last message value
    std::string last_msg_value_;
};

class ExecutionCheck : public BT::ConditionNode {
public:
    ExecutionCheck(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("node_name"),
            BT::InputPort<int>("threshold")  // Threshold in seconds
        };
    }
};
class GlobalExecutionCheck : public BT::ConditionNode {
public:
    GlobalExecutionCheck(const std::string& name, const BT::NodeConfig& config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("threshold")  // Threshold in seconds
        };
    }
};



#endif  // MY_BT_NODES_H
