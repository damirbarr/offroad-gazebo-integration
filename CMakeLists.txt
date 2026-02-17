cmake_minimum_required(VERSION 3.8)
project(offroad_gazebo_integration)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Install Python modules
install(DIRECTORY src/gazebo_adapter
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS src/gazebo_adapter/adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_adapter_node
)
install(PROGRAMS src/gazebo_adapter/udp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME udp_bridge
)
install(PROGRAMS src/gazebo_adapter/bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bridge
)
install(PROGRAMS src/gazebo_adapter/terrain.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME terrain
)
install(PROGRAMS src/gazebo_adapter/sensor_converter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_converter
)
install(PROGRAMS src/gazebo_adapter/laserscan_to_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME laserscan_to_pointcloud
)
install(PROGRAMS src/gazebo_adapter/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install models (if any)
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install meshes
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Python tests
  ament_add_pytest_test(test_adapter test/test_adapter.py)
  ament_add_pytest_test(test_terrain test/test_terrain.py)
endif()

ament_package()
