cmake_minimum_required(VERSION 3.0.2)
project(data_distribute)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
)

  generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )
add_compile_options(-g -std=c++11)    #-g -> core dumped


include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)

catkin_package(
  CATKIN_DEPENDS roscpp  
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)

#add_executable(data_receive src/data_receive.cpp)
#target_link_libraries(data_receive ${catkin_LIBRARIES})

#add_executable(data_send src/data_send.cpp)
#target_link_libraries(data_send ${catkin_LIBRARIES})
