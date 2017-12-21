cmake_minimum_required(VERSION 2.8.3)
project(logical_occlusion_plugin)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

## Enable exception handling for segfaults
set(CMAKE_CXX_FLAGS "-rdynamic -g -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package (gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  ttb_msgs
  system_config
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES   
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp ttb_msgs system_config
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/LogicalOcclusionPlugin.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${GAZEBO_LIBRARIES} )
