cmake_minimum_required(VERSION 2.8.3)
project(lidar_to_mavros)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
#link_libraries(
#   lib
#   ${catkin_LIB_DIRS}
# )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_topic
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs  
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_to_mavros src/lidar_to_mavros.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(lidar_to_mavros
  ${catkin_LIBRARIES}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
## Mark executables and/or libraries for installation
install(TARGETS  lidar_to_mavros    
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
