cmake_minimum_required(VERSION 2.8.3)
project(ros_ogre)

###################################################################
# ROS related stuff                                               #
###################################################################
find_package(catkin REQUIRED COMPONENTS
  	sensor_msgs
	roscpp
	std_msgs
	image_transport
	message_generation
	pcl_conversions
  	pcl_ros
)

#add_message_files(FILES
#	distPose.msg
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS message_runtime
)

###################################################################
# OGRE related stuff                                              #
###################################################################

set(CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules;${CMAKE_MODULE_PATH}")
set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/")

set(CMAKE_DEBUG_POSTFIX "_d")
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")

find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)

include_directories(
	include
	dist/bin
	dist/media 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
	${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${OGRE_SAMPLES_INCLUDEPATH}
)

set(HDRS
	./include/BaseApplication.h
	./include/TutorialApplication.h
	./include/triangulation/triangulation.h
)
 
set(SRCS
	./src/BaseApplication.cpp
	./src/TutorialApplication.cpp
	./include/triangulation/triangulation.cpp
)

add_executable(OgreApp WIN32 ${HDRS} ${SRCS})
 
set_target_properties(OgreApp PROPERTIES DEBUG_POSTFIX _d)
 
target_link_libraries(OgreApp ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${catkin_LIBRARIES} "/usr/lib/x86_64-linux-gnu/libboost_system.so.1.54.0" libvtkCommon.so libvtkFiltering.so libvtkGraphics.so)
 
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/bin)
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/dist/media)
 

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/dist/bin)
 
install(TARGETS OgreApp
	RUNTIME DESTINATION bin
	CONFIGURATIONS All)
