cmake_minimum_required(VERSION 3.0.2)
project(complete_mission)

## 查找 OpenCV 库
find_package(OpenCV REQUIRED)

## 查找 catkin 宏以及依赖库
find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    roscpp
    rospy
    std_msgs
    mavros_msgs
    mavros_extras
    sensor_msgs
    nav_msgs
    geometry_msgs
    mavros
    message_generation
    tf
    serial
    actionlib
    move_base_msgs
    cv_bridge
    geographic_msgs
    yolov8_ros_msgs
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs sensor_msgs nav_msgs geometry_msgs tf serial actionlib move_base_msgs cv_bridge geographic_msgs yolov8_ros_msgs
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  /home/cwkj/cwkj_ws/src/complete_mission/lib
)

add_executable(complete_mission src/complete_mission.cpp)

# 添加目标链接库（关键修改）
target_link_libraries(complete_mission 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    lib
)