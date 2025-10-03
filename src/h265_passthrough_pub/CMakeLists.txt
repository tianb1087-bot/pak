cmake_minimum_required(VERSION 3.8)
project(h265_passthrough_pub LANGUAGES CXX)

# 警告级别
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# FFmpeg via pkg-config
find_package(PkgConfig REQUIRED)
# 分开找模块，尤其要把 libswscale 加上
pkg_check_modules(AVFORMAT REQUIRED IMPORTED_TARGET libavformat)
pkg_check_modules(AVCODEC  REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL   REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE  REQUIRED IMPORTED_TARGET libswscale)

add_executable(h265_passthrough_pub src/h265_passthrough_pub.cpp)
target_compile_features(h265_passthrough_pub PRIVATE cxx_std_17)

# ament 依赖
ament_target_dependencies(h265_passthrough_pub rclcpp sensor_msgs)

# 统一使用 "plain" 签名链接（与 ament 兼容）
target_link_libraries(h265_passthrough_pub
  PkgConfig::AVFORMAT
  PkgConfig::AVCODEC
  PkgConfig::AVUTIL
  PkgConfig::SWSCALE
)

# 保险：补充 include / lib 目录（可选）
target_include_directories(h265_passthrough_pub PRIVATE
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)
target_link_directories(h265_passthrough_pub PRIVATE
  ${AVFORMAT_LIBRARY_DIRS}
  ${AVCODEC_LIBRARY_DIRS}
  ${AVUTIL_LIBRARY_DIRS}
  ${SWSCALE_LIBRARY_DIRS}
)

# 安装
install(TARGETS h265_passthrough_pub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

