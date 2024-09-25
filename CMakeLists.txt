cmake_minimum_required(VERSION 3.16)
project(oasis_visualizer)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17) # 至少c++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED) # cv_bridge 依赖ament_cmake 包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Find Pangolin and Qt5
# find_package(Pangolin REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
# Include directories
include_directories(
  # include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  # ${Pangolin_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
  # ${cv_bridge_INCLUDE_DIRS}
  )
  
  # Source files
  set(SOURCES
  src/main.cpp
  # src/visualizer.cpp
  src/pointcloudviz.cpp
  src/imageviz.cpp
  )
  
  option(LOG "log" on)
  if(LOG)
  add_definitions(-DLOG) # 添加预处理宏定义
  message(STATUS "LOG is ON")
  else()
  message(STATUS "LOG is OFF")
  endif()
  
  # Executable
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
  add_executable(${PROJECT_NAME} ${SOURCES})
  
  ament_target_dependencies(${PROJECT_NAME} cv_bridge rclcpp sensor_msgs) # cv_bridge 依赖ament_cmake 包 必须在ament_target_dependencies
  
  # Link libraries
  target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::OpenGL
  pthread

)

# Install executable
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

