cmake_minimum_required(VERSION 3.8)
project(fanuc_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fanuc_eth_ip REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(fanuc_srvs_msgs REQUIRED)

include_directories(/usr/local/include/EIPScanner)

add_executable(fanuc_srvs src/fanuc_srvs.cpp)
ament_target_dependencies(fanuc_srvs fanuc_srvs_msgs rclcpp std_srvs  fanuc_eth_ip)

install(TARGETS
  fanuc_srvs
  DESTINATION lib/${PROJECT_NAME})

ament_package()
