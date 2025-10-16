cmake_minimum_required(VERSION 3.0.2)
project(relocalization_toolbox_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib_msgs
  message_generation
)

add_service_files(
  FILES
  relocalization_request.srv
  initial_pose_request.srv
)

add_action_files(
  DIRECTORY action
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime
)
