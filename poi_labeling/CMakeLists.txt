cmake_minimum_required(VERSION 2.8.3)
project(poi_labeling)

add_compile_options(-std=c++11)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poi_labeling
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${GAZEBO_INCLUDE_DIRS}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC include/poi_labeling/POILabeling.h)

add_library(poi_labeling SHARED ${headers_MOC}
  src/POILabeling.cpp)

target_link_libraries(poi_labeling ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})

