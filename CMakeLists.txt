cmake_minimum_required(VERSION 3.5)
project(rtsp_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# GStreamer libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)

add_executable(rtsp_camera_node src/rtsp_camera_node.cpp)

ament_target_dependencies(rtsp_camera_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

target_link_libraries(rtsp_camera_node
  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

install(TARGETS rtsp_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
