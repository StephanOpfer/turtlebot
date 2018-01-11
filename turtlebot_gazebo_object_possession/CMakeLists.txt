cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_gazebo_object_possession)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  ttb_msgs
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp ttb_msgs
)


###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ObjectPossessionPlugin.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
