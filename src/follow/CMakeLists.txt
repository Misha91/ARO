cmake_minimum_required(VERSION 2.8.3)
project(follow)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(src)

add_executable(follow src/follow.cpp)
target_link_libraries(follow ${catkin_LIBRARIES})

