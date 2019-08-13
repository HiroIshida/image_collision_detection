cmake_minimum_required(VERSION 2.8.3)
project(image_collision_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  pcl_ros
  std_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/image_dif.cpp)
add_executable(change_detector src/attention.cpp src/image_dif.cpp)
target_link_libraries(change_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_definitions(-std=c++11)
