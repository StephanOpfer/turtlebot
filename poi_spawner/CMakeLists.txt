cmake_minimum_required(VERSION 2.8.3)
project(poi_spawner)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  gazebo_plugins
  gazebo_ros
  roscpp
  system_config
)
    
find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poi_spawner
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp system_config
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/POISpawnerPlugin.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

