# **m_behavior_tree**

### 📜 **Description**
`m_behavior_tree` is a ROS2 package that implements a behavior tree using **BehaviorTree.CPP** and **BehaviorTree.ROS2** to control robot actions dynamically. It allows for modular and reactive robot behaviors by utilizing custom action nodes, condition nodes, and ROS services.

This behavior tree orchestrates the following robot actions:
- **TrackFace**: Tracks a detected human face.
- **Idle**: Switches to an idle state when no face is detected.
- **Yawn**: Executes a "yawn" motion if the robot becomes bored.

---

### 🚀 **Features**
- **Behavior Tree Nodes**:
  - **Custom Action Nodes**:
    - `TrackFace`: Reactively tracks a face based on camera input.
    - `Idle`: Sets the robot to an idle state when no face is detected.
    - `Yawn`: Triggers a yawn action if the robot gets bored.
  - **Custom Condition Nodes**:
    - `IsDetectedCondition`: Checks if a face is detected.
    - `IsBoredCondition`: Checks if the robot is bored based on a ROS topic.
- **Reactive Fallback Design**: Ensures smooth transitions between states like `Idle` and `TrackFace`.
- **ROS2 Service Integration**: Uses ROS services for motion execution and state transitions.

---

### 🗂️ **Behavior Tree Structure**
The behavior tree is defined in [my_behavior_tree.xml](config/my_behavior_tree.xml).


**Node Workflow**:
- **Face Detected**: The tree activates the `TrackFace` node.
- **No Face Detected**: The tree switches to `Idle`.
- **Bored Condition**: If the robot becomes bored, it triggers the `Yawn` action.

---

### 📦 **Dependencies**
The package depends on the following ROS2 and external libraries:
- `rclcpp`
- `std_msgs`
- `behaviortree_cpp`
- `behaviortree_ros2`
- `custom_interfaces` (for service definitions)

Make sure these dependencies are installed in your ROS2 workspace.

---

### ⚙️ **Setup Instructions**

#### Prerequisites
1. **ROS2 Rolling** installed ([ROS2 installation guide](https://docs.ros.org/en/rolling/Installation.html)).
2. BehaviorTree.CPP and BehaviorTree.ROS2 installed in your workspace.

#### Build the Package
```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src
# Clone the repository
git clone <repository_url> m_behavior_tree
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select m_behavior_tree

