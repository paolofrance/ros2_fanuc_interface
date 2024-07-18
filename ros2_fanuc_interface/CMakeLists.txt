cmake_minimum_required(VERSION 3.8)
project(ros2_fanuc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(fanuc_eth_ip REQUIRED)
find_package(fanuc_rmi REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED)


include_directories(/usr/local/include/EIPScanner)
include_directories(include srv)
include_directories( ${Boost_INCLUDE_DIRS} )

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  fanuc_eth_ip
  fanuc_rmi
  Boost
)

add_library(fanuc SHARED  src/fanuc_hw.cpp)

target_compile_features(fanuc PUBLIC cxx_std_17)
target_include_directories(fanuc PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/fanuc>
)

ament_target_dependencies(
  fanuc 
  ${HW_IF_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface hw_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/fanuc
)

install(TARGETS fanuc
  EXPORT fanuc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(dpm src/dpm.cpp)

ament_target_dependencies(
  dpm 
  ${HW_IF_INCLUDE_DEPENDS}
  std_srvs
)
install(TARGETS
dpm
DESTINATION lib/${PROJECT_NAME})

ament_export_targets(fanuc HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})

ament_package()
