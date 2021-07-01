cmake_minimum_required(VERSION 3.1)
project(i3dr_halcon_object_detect)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Halcon)
if(NOT ${Halcon_FOUND})
   message(FATAL_ERROR  "Skip processing ${PROJECT_NAME}, because HALCON library is missing!")
   return()
endif()

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
    cv_bridge
)

find_package(OpenCV 3 REQUIRED)

include_directories(
	include
	${Halcon_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs
	LIBRARIES ${PROJECT_NAME}
)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/bin
)

add_executable(
    detect src/detect.cpp
)

target_link_libraries(
    detect
	${catkin_LIBRARIES}
	${Halcon_LIBRARIES}
	${OpenCV_LIBRARIES}
)

add_dependencies(detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})