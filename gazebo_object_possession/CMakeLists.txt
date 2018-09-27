cmake_minimum_required(VERSION 2.8.3)
project(gazebo_object_possession)

cmake_policy(SET CMP0054 NEW)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  gazebo_plugins
  gazebo_ros
  system_config
  ttb_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_object_possession
  CATKIN_DEPENDS gazebo_plugins gazebo_ros message_runtime system_config ttb_msgs
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ObjectPossession.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(gazebo_object_possession ttb_msgs_generate_messages_cpp)

