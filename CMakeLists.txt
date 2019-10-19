cmake_minimum_required(VERSION 2.8.3)
project(tether_dynamics)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
if (NOT TARGET utils)
  include_directories(lib/utils/include)
  add_subdirectory(lib/utils)
endif()

cs_add_executable(tether_dynamics_node
  src/tether_dynamics_node.cpp
  src/tether_dynamics.cpp
)
target_link_libraries(tether_dynamics_node utils ${catkin_LIBRARIES})

cs_install()
cs_export()
