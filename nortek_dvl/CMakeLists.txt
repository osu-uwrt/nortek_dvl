cmake_minimum_required(VERSION 3.8)
project(nortek_dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nortek_dvl_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(dvl src/dvl.cpp src/dvl_driver.cpp src/async_tcp.cpp)

#trace include dirs
target_include_directories(dvl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/interfaces>
  $<INSTALL_INTERFACE:include>)

target_compile_features(dvl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(dvl  
  rclcpp
  nortek_dvl_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS dvl
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
