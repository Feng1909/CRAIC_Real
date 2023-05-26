cmake_minimum_required(VERSION 3.0.2)
project(fast_perching)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  traj_opt
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  vis_utils
)

catkin_package(
 INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
 LIBRARIES
 CATKIN_DEPENDS traj_opt
 DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(fast_perching 
  src/main.cpp
  src/fast_perching.cpp
  src/fast_perching_handle.cpp
)

target_link_libraries(
  fast_perching
  ${catkin_LIBRARIES}
)
