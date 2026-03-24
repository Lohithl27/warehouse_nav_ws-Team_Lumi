cmake_minimum_required(VERSION 3.8)
project(warenav_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package (nodes)
ament_python_install_package(${PROJECT_NAME})

# Install executable nodes
install(PROGRAMS
  warenav_nav/frontier_explorer.py
  warenav_nav/mission_controller.py
  warenav_nav/obstacle_detector.py
  warenav_nav/inventory_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
