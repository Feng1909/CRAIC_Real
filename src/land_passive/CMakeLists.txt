cmake_minimum_required(VERSION 3.0.2)
project(offboard_pkg)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf2_eigen
  tf
  visualization_msgs
  pcl_ros
)

find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   
   mavros_msgs
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard_pkg
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)
link_directories(${PCL_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

 add_executable(multi_aruco_track_circle_fit_land src/main.cpp)
#  add_dependencies(multi_aruco_track_circle_fit_land offboard_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
 target_link_libraries(multi_aruco_track_circle_fit_land ${catkin_LIBRARIES} ${CERES_LIBRARIES})
