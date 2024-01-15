cmake_minimum_required(VERSION 3.8)
project(ros2_fanuc_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)


include_directories(include)

add_library(fanuc SHARED
  src/fanuc_topic.cpp
)

target_compile_features(fanuc PUBLIC cxx_std_17)
target_include_directories(fanuc PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/fanuc>
)
ament_target_dependencies(
  fanuc PUBLIC
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

ament_export_targets(fanuc HAS_LIBRARY_TARGET)

install(DIRECTORY scripts launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(fanuc_eth_ip src/fanuc_eth_ip.cpp)

include_directories(/usr/local/include/EIPScanner)
target_link_libraries(fanuc_eth_ip EIPScanner)

install(TARGETS
fanuc_eth_ip
DESTINATION lib/${PROJECT_NAME})



# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/ros2_fanuc_interface.py
  scripts/ros2_fanuc_fake_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
