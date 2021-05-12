cmake_minimum_required(VERSION 3.5)
project(neato_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(neato_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(neato_gazebo_diff_drive SHARED
  src/neato_gazebo_diff_drive.cpp
)
ament_target_dependencies(neato_gazebo_diff_drive
  gazebo_dev
  gazebo_ros
  nav_msgs
  neato_msgs
  rclcpp
  tf2_ros
)
ament_export_libraries(neato_gazebo_diff_drive)
install(TARGETS
  neato_gazebo_diff_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif(BUILD_TESTING)

ament_package()
