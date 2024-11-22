#ifndef MY_BT_NODES_H
#define MY_BT_NODES_H

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// Custom Action Node: TrackFace
class TrackFace : public BT::RosTopicPubNode<std_msgs::msg::Bool> {
public:
    TrackFace(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
protected:
    bool setMessage(std_msgs::msg::Bool& msg) override;
};


class Idle : public BT::RosTopicPubNode<std_msgs::msg::Bool> {
public:
    Idle(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
protected:
    bool setMessage(std_msgs::msg::Bool& msg) override;
};


// Custom Action Node: Idle
// class Idle : public BT::SyncActionNode {
// public:
//     Idle(const std::string& name);
//     BT::NodeStatus tick() override;
// };

// Custom Condition Node: IsDetectedCondition
class IsDetectedCondition : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
public:
    IsDetectedCondition(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& last_msg) override;
    static BT::PortsList providedPorts();
};

#endif  // MY_BT_NODES_H
