cmake_minimum_required(VERSION 3.0.2)
project(relocalization_toolbox)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -flto -ffast-math -funroll-loops)
add_link_options(-flto)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_generation
  genmsg
  actionlib
  actionlib_msgs
  pcl_ros
  cv_bridge
  relocalization_toolbox_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    tf2
    tf2_ros
    tf2_sensor_msgs
    message_runtime
    relocalization_toolbox_msgs
  DEPENDS
    EIGEN3
    PCL
    OpenCV
)

add_library(relocalization_toolbox_2d_deps
  src/utils/math_utils.cpp
  src/utils/2d/likelihood_field_2d.cpp
  src/utils/2d/sector_query_2d.cpp
  src/utils/2d/submap_set_2d.cpp
  src/utils/2d/gicp_2d.cpp
  src/utils/2d/map_io_2d.cpp
  src/utils/2d/sampling_2d.cpp
  src/utils/2d/relocalization_2d.cpp
)

target_link_libraries(relocalization_toolbox_2d_deps
  PRIVATE
    OpenMP::OpenMP_CXX
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)

add_executable(relocalization_2d_handler
  src/relocalization_2d_handler.cpp
)

target_link_libraries(relocalization_2d_handler
  relocalization_toolbox_2d_deps
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
