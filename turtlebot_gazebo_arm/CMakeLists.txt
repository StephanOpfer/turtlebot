cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_gazebo_arm)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

add_message_files(
  FILES
  GrabDropObject.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp message_runtime
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

add_library(${PROJECT_NAME} src/ArmPlugin.cpp )
add_dependencies(turtlebot_gazebo_arm turtlebot_gazebo_arm_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
