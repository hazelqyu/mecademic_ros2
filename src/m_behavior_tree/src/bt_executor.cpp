#include <chrono>
#include <memory>
#include <atomic>
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "m_behavior_tree/bt_nodes.h"

using namespace std::chrono_literals;

class BTExecutor : public rclcpp::Node
{
public:
  BTExecutor()
  : Node("bt_executor_node"), running_(false){}

  void init_behavior_tree()
  {
    if (running_) {
      RCLCPP_WARN(this->get_logger(), "Behavior tree is already running. Skipping reinitialization.");
      return;
    }

    BT::RosNodeParams params;
    params.nh = this->shared_from_this();  // Use shared_from_this to get a shared_ptr

    // Register BehaviorTree nodes
    factory_.registerNodeType<IsAwakeCondition>("IsAwakeCondition", params);
    factory_.registerNodeType<Asleep>("Asleep");

    factory_.registerNodeType<IsDetectedCondition>("IsDetectedCondition", params);
    factory_.registerNodeType<TrackFace>("TrackFace");
    factory_.registerNodeType<Idle>("Idle");

    factory_.registerNodeType<IsBoredCondition>("IsBoredCondition", params);
    factory_.registerNodeType<ExecutionCheck>("ExecutionCheck");
    factory_.registerNodeType<Yawn>("Yawn");

    factory_.registerNodeType<IsAlertCondition>("IsAlertCondition", params);
    factory_.registerNodeType<Alert>("Alert");

    factory_.registerNodeType<IsHappyCondition>("IsHappyCondition", params);
    factory_.registerNodeType<Dance>("Dance");

    // Load behavior tree from XML
    tree_ = factory_.createTreeFromFile("/home/andrek/ros2_ws/src/m_behavior_tree/config/my_behavior_tree.xml");

    // Start the timer for periodic execution
    timer_ = this->create_wall_timer(
      10ms,
      std::bind(&BTExecutor::tick_tree, this)  // Timer callback
    );

    running_ = true;  // Mark the tree as running
    RCLCPP_INFO(this->get_logger(), "Behavior tree initialized and execution started.");
  }

  ~BTExecutor()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down BTExecutor...");
    if (timer_) {
      timer_->cancel();  // Cancel the timer to stop periodic callbacks
    }
    running_ = false;
    RCLCPP_INFO(this->get_logger(), "BTExecutor shutdown complete.");
  }

private:
  void tick_tree()
  {
    if (!rclcpp::ok() || !running_) {
      RCLCPP_INFO(this->get_logger(), "Stopping behavior tree execution.");
      timer_->cancel();  // Stop the timer to avoid further callbacks
      return;
    }

    // Execute the behavior tree
    tree_.tickWhileRunning();
    RCLCPP_DEBUG(this->get_logger(), "Ticking the behavior tree...");
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> running_;  // Flag for thread-safe shutdown
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto bt_executor_node = std::make_shared<BTExecutor>();

  // Initialize the behavior tree after node construction
  bt_executor_node->init_behavior_tree();

  RCLCPP_INFO(bt_executor_node->get_logger(), "Node spinning. Press Ctrl+C to exit.");

  // Spin to process callbacks
  try {
    rclcpp::spin(bt_executor_node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(bt_executor_node->get_logger(), "Exception caught: %s", e.what());
  }

  // Ensure proper cleanup
  rclcpp::shutdown();
  RCLCPP_INFO(bt_executor_node->get_logger(), "Node has shut down.");

  return 0;
}
