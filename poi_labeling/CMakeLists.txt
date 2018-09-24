cmake_minimum_required(VERSION 2.8.3)
project(poi_labeling)

add_compile_options(-std=c++11)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED qt_gui)

find_package(Qt5Core REQUIRED)
get_target_property(Qt5Core_location Qt5::Core LOCATION)
find_package(Qt5Gui REQUIRED)
get_target_property(Qt5Gui_location Qt5::Gui LOCATION)
find_package(Qt5X11Extras REQUIRED)
get_target_property(Qt5X11Extras_location Qt5::X11Extras LOCATION)
find_package(Qt5Widgets REQUIRED)
get_target_property(Qt5Widgets_location Qt5::Widgets LOCATION)
SET( QT_WRAP_CPP TRUE )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poi_labeling
  CATKIN_DEPENDS qt_gui
#  DEPENDS system_lib
)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Gui_INCLUDE_DIRS}
	${Qt5X11Extras_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIR} 
	${GAZEBO_INCLUDE_DIRS}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT5_WRAP_CPP(headers_MOC include/poi_labeling/POILabeling.h)

add_library(${PROJECT_NAME} SHARED ${headers_MOC}
  src/POILabeling.cpp)

target_link_libraries(${PROJECT_NAME} 
	${GAZEBO_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${Qt5Core_location}
	${Qt5Gui_location}
	${Qt5X11Extras_location}
	${Qt5Widgets_location}
)

