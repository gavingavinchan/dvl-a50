# Product Requirement Document: DVL Dead Reckoning Visualization

## 1. Introduction

This document outlines the requirements for a new feature in the `dvl-a50` ROS2 package. The goal is to process the dead reckoning data from the Water Linked DVL-A50 and publish it in a format that can be easily visualized in RViz2. This will provide users with immediate visual feedback of the DVL's estimated trajectory, which is crucial for debugging, monitoring, and integrating the sensor into broader navigation systems.

## 2. Problem Statement

The `dvl-a50` package currently publishes the dead reckoning position on the `/dvl/position` topic using the custom `dvl_msgs/msg/DVLDR` message type. This format is not directly visualizable in RViz2 as a path or trajectory. Users who want to visualize the DVL's path in RViz2 must create their own nodes to subscribe to the `/dvl/position` topic and republish it as a `nav_msgs/msg/Path` message. This creates redundant work and a barrier to easy use.

## 3. Goals and Objectives

*   **Primary Goal:** Enable users to visualize the DVL-A50's dead reckoning trajectory in RViz2 out-of-the-box.
*   **Objectives:**
    *   Create a new, separate Python-based ROS2 node.
    *   This node will subscribe to the existing `/dvl/position` topic.
    *   It will then republish the trajectory on a new topic using the `nav_msgs/msg/Path` message type.
    *   Ensure the implementation is efficient and follows the project's existing conventions.
    *   Provide configuration options to customize topic names and frame IDs.
    *   The new node should be launched from the user's existing `main_launch/launch/playback.launch.py` file.

## 4. User Profile

The target users for this feature are robotics engineers and developers using the DVL-A50 sensor with ROS2 for applications such as:
*   Underwater vehicle navigation and control.
*   Localization and mapping (SLAM).
*   Sensor data verification and system debugging.

## 5. Requirements

### 5.1. Functional Requirements

*   **Node Implementation:**
    *   A new Python ROS2 node shall be created.
    *   The node shall be named `position_to_path_node.py` and located in the `scripts` directory.

*   **Topic Subscription:**
    *   The node shall subscribe to the `/dvl/position` topic, which has a message type of `dvl_msgs/msg/DVLDR`.

*   **Topic Publication:**
    *   The node shall publish the trajectory on a new ROS topic.
    *   **Default Topic Name:** `/dvl/path`
    *   **Message Type:** `nav_msgs/msg/Path`

*   **Coordinate Frames:**
    *   The published `nav_msgs/msg/Path` message must contain a `frame_id` in its header.
    *   The `frame_id` shall be configurable via a ROS parameter.
    *   **Default `frame_id`:** `odom`

*   **Launch Integration:**
    *   The `position_to_path_node.py` will be launched from the user's `main_launch/launch/playback.launch.py` file.

### 5.2. Non-Functional Requirements

*   **Performance:** The new node should be lightweight and introduce minimal latency.
*   **Documentation:** The `README.md` file must be updated to document the new node, the published topic, and all related ROS parameters.

## 6. Out of Scope

*   **Modification of Existing Nodes:** This feature will not modify the existing `dvl_a50.py` or C++ nodes.
*   **Dead Reckoning Calculation:** The new node will not perform any dead reckoning calculations. It will only transform and republish the data.
*   **TF Broadcasting:** The node will not broadcast transforms (`tf2`) between coordinate frames. It will only publish the path message.

## 7. Success Criteria

*   A new Python node `position_to_path_node.py` is created.
*   When the `main_launch/launch/playback.launch.py` is launched, the `position_to_path_node.py` is started.
*   The node subscribes to `/dvl/position` and publishes to `/dvl/path`.
*   The `/dvl/path` topic uses the `nav_msgs/msg/Path` message type.
*   In RViz2, adding a "Path" display and subscribing to `/dvl/path` correctly visualizes the trajectory of the DVL.
*   The feature is documented in the project's `README.md`.

## 8. Usage Instructions for RViz2 Visualization

To visualize the DVL's dead reckoning path in RViz2, follow these steps:

1.  **Start the `static_transform_publisher`:**
    Open a new terminal and run the following command. This is necessary to provide a transform from the `odom` frame (where the path is published) to a `base_link` frame, which RViz2 often expects.
    ```bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
    ```

2.  **Run the DVL data source:**
    Ensure your DVL data is being published. This could be by running the live DVL node or playing a rosbag:
    *   **Live DVL:** `ros2 run dvl_a50 dvl_a50.py --ros-args -p ip_address:='192.168.194.95'` (or your C++ node)
    *   **Rosbag Playback:** `ros2 launch main_launch playback.launch.py bag_file:=~/ros2_ws/bags/session_2025-06-10_21-20-37`

3.  **Start RViz2:**
    Open another new terminal and run:
    ```bash
    rviz2
    ```

4.  **Configure RViz2:**
    *   **Fixed Frame:** In the "Displays" panel (left side), set the "Fixed Frame" to `odom`.
    *   **Add Path Display:**
        *   Click the "Add" button (green plus sign).
        *   Search for and select "Path" under `rviz_default_plugins`. Click "OK".
        *   In the newly added "Path" display:
            *   Set the "Topic" to `/dvl/path`.
            *   Set the "Buffer Length" to `1000` (or a higher value if you want to see a longer history).
    *   **Zoom Out:** You may need to zoom out significantly in the 3D view to see the entire trajectory, especially if the DVL has moved a large distance.

By following these steps, you should be able to visualize the DVL's dead reckoning path in RViz2.