cmake_minimum_required(VERSION 3.29.5)
project(vk_slam3d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES vk_slam3d
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(vk_slam3d 
  src/vk_slam3d.cpp
  src/graphbasedslam.cpp
  src/voxelgrid.cpp
  src/voxelhashmap.cpp
  src/scanmatcher.cpp
  src/preprocessing.cpp
)

target_link_libraries(vk_slam3d
  ${catkin_LIBRARIES}
  ${Eigen_INCLUDE_DRIS} Boost::thread
)
