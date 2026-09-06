cmake_minimum_required(VERSION 3.8)
project(g2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes maps config rviz launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
