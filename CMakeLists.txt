cmake_minimum_required(VERSION 2.8.3)
project(df)

# Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	image_transport
	sensor_msgs
)

find_package(PkgConfig)

pkg_check_modules(gst gstreamer-1.0)
#pkg_check_modules(gstbase gstreamer-base-1.0)
pkg_check_modules(gstapp gstreamer-app-1.0)

pkg_check_modules(gstsdp gstreamer-sdp-1.0)
pkg_check_modules(gstwebrtc gstreamer-webrtc-1.0)
pkg_check_modules(libsoup libsoup-2.4)
pkg_check_modules(jsonglib json-glib-1.0)
#pkg_check_modules(glib glib-2.0)


#message(STATUS "inc: ${gst_INCLUDE_DIRS}")
#message(STATUS "lib: ${gst_LIBRARIES}")
#message(STATUS "link_lib: ${gst_LIBRARY_DIRS}")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  #LIBRARIES df
  #CATKIN_DEPENDS roscpp rospy std_msgs image_transport sensor_msgs
  #DEPENDS gst gstsdp gstwebrtc libsoup jsonglib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gst_INCLUDE_DIRS}
  ${gstapp_INCLUDE_DIRS}
  ${gstsdp_INCLUDE_DIRS}
  ${gstwebrtc_INCLUDE_DIRS}
  ${libsoup_INCLUDE_DIRS}
  ${jsonglib_INCLUDE_DIRS}
)

link_directories(
	${libsoup_LIBRARY_DIRS}
	${jsonglib_LIBRARY_DIRS}
	${gstsdp_LIBRARY_DIRS}
	${gstapp_LIBRARY_DIRS}
	${gstwebrtc_LIBRARY_DIRS}
	${gst_LIBRARY_DIRS}
)

add_executable(webrtcrs
  src/webrtcrealsense.cpp
)

target_link_libraries(webrtcrs
	${catkin_LIBRARIES}
	${libsoup_LIBRARIES}
	${jsonglib_LIBRARIES}
	${gstsdp_LIBRARIES}
	${gstapp_LIBRARIES}
	${gstwebrtc_LIBRARIES}
	${gst_LIBRARIES}
)


#add_executable(hello
  #src/test.cpp
#)

#target_link_libraries(hello
	#${catkin_LIBRARIES}
	#${gst_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS webrtcrs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS hello
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_df.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
