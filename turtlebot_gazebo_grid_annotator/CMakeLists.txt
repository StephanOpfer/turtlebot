cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_gazebo_grid_annotator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  ttb_msgs
  system_config
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp ttb_msgs system_config
)

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${ODE_LIBRARY_DIRS})
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Annotator.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
