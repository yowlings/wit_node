cmake_minimum_required(VERSION 2.8.3)
project(wit_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)

find_package(catkin REQUIRED COMPONENTS
  ecl_build
  ecl_devices
  ecl_sigslots
  ecl_time
  ecl_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nodelet
  pluginlib
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ImuGpsRaw.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wit_nodelet
  CATKIN_DEPENDS
  ecl_build
  ecl_devices
  ecl_geometry
  ecl_sigslots
  ecl_time
  nodelet
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
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

add_subdirectory(src)





















