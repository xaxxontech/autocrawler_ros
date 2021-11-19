cmake_minimum_required(VERSION 3.5)
project(autocrawler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED) # required for python 
find_package(rclpy REQUIRED)  # required for python

ament_python_install_package(${PROJECT_NAME}) # required for python


# ---- below required for webrtcrealsense.cpp ----
find_package(PkgConfig)

pkg_check_modules(gst gstreamer-1.0)
pkg_check_modules(gstapp gstreamer-app-1.0)
pkg_check_modules(gstsdp gstreamer-sdp-1.0)
pkg_check_modules(gstwebrtc gstreamer-webrtc-1.0)
pkg_check_modules(libsoup libsoup-2.4)
pkg_check_modules(jsonglib json-glib-1.0)

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

add_executable(webrtcrs src/webrtcrealsense.cpp)
ament_target_dependencies(webrtcrs rclcpp std_msgs image_transport sensor_msgs)

target_link_libraries(webrtcrs
	${catkin_LIBRARIES}
	${libsoup_LIBRARIES}
	${jsonglib_LIBRARIES}
	${gstsdp_LIBRARIES}
	${gstapp_LIBRARIES}
	${gstwebrtc_LIBRARIES}
	${gst_LIBRARIES}
)

install(TARGETS
  webrtcrs
  DESTINATION lib/${PROJECT_NAME}
 )
# ---- end of required for webrtcrealsense.cpp ----


install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ${PROJECT_NAME}/webrtc_status_listener.py
  ${PROJECT_NAME}/docktargettostate.py
  ${PROJECT_NAME}/image_to_shm.py
  ${PROJECT_NAME}/odom_tf.py
  ${PROJECT_NAME}/map_remote.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
