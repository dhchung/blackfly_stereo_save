cmake_minimum_required(VERSION 3.0.2)
project(blackfly_stereo_save)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	image_transport
	roscpp
	sensor_msgs
	std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

FIND_LIBRARY(SPINNAKER_LIBRARIES Spinnaker /opt/spinnaker/lib/)

catkin_package()

file(GLOB CLASS_SOURCES
    "classes/*")

if(EXISTS "/opt/spinnaker")
	include_directories(
		/opt/spinnaker/include
		classes/
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
	)
	add_executable(save_image 
				   src/main.cpp
				   ${CLASS_SOURCES})

	target_link_libraries(save_image
                          ${catkin_LIBRARIES} 
                          ${SPINNAKER_LIBRARIES} 
                          ${OpenCV_LIBRARIES})
else()
	message("Saesangeh, Spinnaker is not installed")
endif()