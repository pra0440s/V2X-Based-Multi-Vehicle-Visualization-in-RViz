cmake_minimum_required(VERSION 3.5)
project(vehicle_visualizer)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install 3D model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()

