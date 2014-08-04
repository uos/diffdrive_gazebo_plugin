# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(diffdrive_gazebo_plugin)
# Load catkin and all dependencies required for this package

find_package(catkin REQUIRED COMPONENTS gazebo_ros nav_msgs geometry_msgs sensor_msgs)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include/)

catkin_package(
    CATKIN_DEPENDS gazebo_ros nav_msgs geometry_msgs sensor_msgs
    INCLUDE_DIRS include
)

add_definitions(-fPIC)
add_library(gazebo_ros_diffdrive_uos src/gazebo_ros_diffdrive_uos.cpp)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
