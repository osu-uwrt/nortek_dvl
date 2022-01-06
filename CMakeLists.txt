cmake_minimum_required(VERSION 3.8)
project(nortek_dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# build tacopie library
OPTION(BUILD_EXAMPLES "examples for taco pie" OFF)
add_subdirectory(tacopie)

add_executable(dvl src/dvl.cpp src/dvl_driver.cpp)
target_include_directories(dvl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/tacopie/includes)
target_compile_features(dvl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
add_dependencies(dvl tacopie)
target_link_libraries(dvl tacopie)

ament_target_dependencies(dvl  
  rclcpp
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

#generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Dvl.msg"
  "msg/DvlStatus.msg"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(dvl
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
