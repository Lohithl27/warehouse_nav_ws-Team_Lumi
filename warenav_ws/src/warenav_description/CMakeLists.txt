cmake_minimum_required(VERSION 3.8)
project(warenav_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf worlds launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
