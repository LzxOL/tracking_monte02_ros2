cmake_minimum_required(VERSION 3.8)
project(robot_video_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g -O0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.5 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

include_directories(
    include
    3rdparty/dds/include
    3rdparty/yllog/include
    3rdparty/zdepth/include
    3rdparty/live555/include
    3rdparty/live555/include/liveMedia
    3rdparty/live555/include/groupsock
    3rdparty/live555/include/UsageEnvironment
    3rdparty/live555/include/BasicUsageEnvironment
    rtsp_client
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVCODEC_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${AVFORMAT_LIBRARY_DIRS}
    ${AVCODEC_LIBRARY_DIRS}
    ${AVUTIL_LIBRARY_DIRS}
    ${SWSCALE_LIBRARY_DIRS}
    3rdparty/dds/lib
    3rdparty/yllog/lib
    3rdparty/zdepth/lib
    3rdparty/live555/lib
)

message("AVFORMAT_LIBRARY_DIRS: ${AVFORMAT_LIBRARY_DIRS}")
message("AVCODEC_LIBRARY_DIRS: ${AVCODEC_LIBRARY_DIRS}")
message("AVUTIL_LIBRARY_DIRS: ${AVUTIL_LIBRARY_DIRS}")
message("SWSCALE_LIBRARY_DIRS: ${SWSCALE_LIBRARY_DIRS}")

set(ROBOT_VIDEO_CLIENT_SRC
    src/main.cpp
    src/video_client_node.cpp
    src/video_decoder.cpp
    src/depth_decoder.cpp
    src/utils.cpp
)

file(GLOB RTSP_CLIENT_SRC "rtsp_client/*.cpp")

add_executable(robot_video_client
    ${ROBOT_VIDEO_CLIENT_SRC}
    ${RTSP_CLIENT_SRC}
)

target_link_libraries(robot_video_client
  ${AVFORMAT_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  ${SWSCALE_LIBRARIES}
  ${OpenCV_LIBRARIES}
  liveMedia
  BasicUsageEnvironment
  UsageEnvironment
  groupsock
  ssl
  crypto
  yhdds
  yllog
  zdepth
  zstd
  pthread
)

ament_target_dependencies(robot_video_client rclcpp std_msgs sensor_msgs)

add_executable(video_click_node
    src/video_click_node.cpp
)

target_link_libraries(video_click_node
    ${OpenCV_LIBRARIES}
)

ament_target_dependencies(video_click_node rclcpp sensor_msgs cv_bridge)

install(TARGETS
  robot_video_client
  video_click_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 3rdparty/dds/lib/
  DESTINATION lib/
)

install(DIRECTORY 3rdparty/yllog/lib/
  DESTINATION lib/
)

install(DIRECTORY 3rdparty/live555/lib/
  DESTINATION lib/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()