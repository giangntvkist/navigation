cmake_minimum_required(VERSION 3.0.2)
# cmake_minimum_required(VERSION 3.29.5)
project(human_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
# find_package(Matplot++ REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES human_tracking
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(human_tracking src/human_tracking.cpp src/ekf.cpp src/imm_filter.cpp)
target_link_libraries(human_tracking ${catkin_LIBRARIES} ${Eigen_INCLUDE_DRIS})
# target_link_libraries(human_tracking ${catkin_LIBRARIES} ${Eigen_INCLUDE_DRIS} Matplot++::matplot)