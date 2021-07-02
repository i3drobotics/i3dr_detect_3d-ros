cmake_minimum_required(VERSION 3.1)
project(i3dr_detect_pcl_primitives)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	tf
	visualization_msgs
	object_recognition_msgs
    cv_bridge
)

find_package(OpenCV 3 REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs
	LIBRARIES ${PROJECT_NAME}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(
    detect_pcl_primitives src/detect_pcl_primitives.cpp
)

target_link_libraries(
    detect_pcl_primitives
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(
    load_ply src/load_ply.cpp
)

target_link_libraries(
    load_ply
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(
    reduce_pcl src/reduce_pcl.cpp
)

target_link_libraries(
    reduce_pcl
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
)

add_dependencies(detect_pcl_primitives ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(load_ply ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reduce_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})