cmake_minimum_required(VERSION 2.8.3)
project(tether_dynamics)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  rosflight_sil
  aerowake_utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs visualization_msgs
    tf2 tf2_ros rosflight_sil aerowake_utils
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tether_dynamics_node
  src/tether_dynamics_node.cpp
  src/tether_dynamics.cpp
)
add_dependencies(tether_dynamics_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tether_dynamics_node
  aerowake-utils-lib ${catkin_LIBRARIES}
)
