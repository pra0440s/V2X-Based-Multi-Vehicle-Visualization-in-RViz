# V2X-Based-Multi-Vehicle-Visualization-in-RViz


## Main Contributor
Pranav Balaji Balachandran 

## Overview
This project is designed to provide real-time visualization of multiple vehicles in a simulated environment using ROS2. It combines localization data from an OptiTrack system for the ego vehicle and V2X CAM messages from nearby vehicles to accurately display their positions and orientations. By visualizing all vehicles in RViz with distinct URDF car models and colors, the system helps developers to monitor vehicle movements, test V2X communication, and analyze traffic interactions in a controlled setup. Overall, this project serves as a platform for testing autonomous vehicle coordination, situational awareness, and V2X-based traffic simulations.

## Table of Contents
- [Architecture Diagram](#architecture-diagram)
- [Component Functionalities](#component-Functionalities)
- [Installation](#installation)
- [License](#license)


## Architecture Diagram
   ```mermaid
   graph LR
    %% Style for topics (rounded rectangles with colored fill and border)
    style OdomTopic stroke:#2980B9,stroke-width:3px,fill:#3498DB,rx:10,ry:10,color:#fff
    style NearbyVehicle stroke:#2980B9,stroke-width:3px,fill:#3498DB,rx:10,ry:10,color:#fff
    style SignalStatus stroke:#2980B9,stroke-width:3px,fill:#3498DB,rx:10,ry:10,color:#fff
    style CAMTopic stroke:#2980B9,stroke-width:3px,fill:#85C1E9,rx:10,ry:10,color:#fff
    style SPATTopic stroke:#2980B9,stroke-width:3px,fill:#85C1E9,rx:10,ry:10,color:#fff

    %% Nodes (components)
    style LocalizationNode stroke:#000,stroke-width:2px,fill:#fff,color:#000
    style V2XNode stroke:#000,stroke-width:2px,fill:#fff,color:#000
    style VisualizerNode stroke:#000,stroke-width:2px,fill:#fff,color:#000
    style RViz stroke:#000,stroke-width:2px,fill:#fff,color:#000

    %% Components
    LocalizationNode["Localization Node"]
    V2XNode["V2X Node"]
    VisualizerNode["Vehicle Visualizer Node"]
    RViz["RViz2"]

    %% Topics
    OdomTopic["/odom (nav_msgs/Odometry)"]
    CAMTopic["/cam (v2x/CAM)"]
    SPATTopic["/spat (v2x/SPATEM)"]
    NearbyVehicle["/nearby_vehicle (geometry_msgs/PoseStamped)"]
    SignalStatus["/signal_status (custom_msg/SignalStatus)"]

    %% Connections
    LocalizationNode --> OdomTopic
    OdomTopic --> VisualizerNode

    CAMTopic --> V2XNode
    SPATTopic --> V2XNode

    V2XNode --> NearbyVehicle
    V2XNode --> SignalStatus

    NearbyVehicle --> VisualizerNode
    VisualizerNode --> RViz
    OdomTopic --> RViz
```


  
## Component Functionality
 - Localization Node
   - Subscribes to `/pose_modelcars` (OptiTrack data).
   - Applies Kalman filtering.
   - Publishes ego car odometry on `/odom`.
 - V2X Node
   - Subscribes to `/cam` messages → extracts vehicle ID, position, heading, and speed.
   - Converts GNSS positions (lat/lon/alt) → ENU (local coordinates).
   - Publishes each nearby vehicle as `/nearby_vehicle` (PoseStamped).
   - Subscribes to `/spatem` messages → Publishes `/signal_status` (traffic light state).
  - Vehicle Visualizer Node
   - Subscribes to `/odom` (ego vehicle pose).
   - Subscribes to `/nearby_vehicle` (other vehicles’ poses).
   - Publishes TF transforms (map → car_X/base_link).
   - Loads URDF/Xacro car models with unique colors.
   - Displays all cars in RViz.
 - RViz
   - Final visualization of ego + nearby vehicles and their motion.
  
## Installation
- Clone the repository into your ROS2 workspace:
cd ~/ros2_ws/src
```git clone https://github.com/<pra0440s>/<V2X-Based-Multi-Vehicle-Visualization-in-RViz>.git```
  - cd ~/ros2_ws
  - colcon build
  - source install/setup.bash

- Open 4 terminals (or combine using a launch file):
  - Run Localization Node
     - ```ros2 run localization_pkg localization_node```
  - Run V2X Node
     - ``` ros2 run v2x_reciever v2x_reciever```
  - Run Vehicle Visualizer
     - ``` ros2 run vehicle_visualizer visualize_cam```
  - Launch RViz
     - ```rviz2```

## License
This project is licensed under the **Apache 2.0 License** - see the [LICENSE](LICENSE) file for details.
