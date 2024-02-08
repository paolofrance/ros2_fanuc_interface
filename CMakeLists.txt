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
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(/usr/local/include/EIPScanner)
include_directories(include)


set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)



add_library(fanuc             SHARED  src/fanuc_hw.cpp)
add_library(fanuc_eth_ip      SHARED  src/fanuc_eth_ip.cpp)
add_library(rmi_communication SHARED  src/rmi_communication.cpp)
add_library(rmi_driver        SHARED  src/rmi_driver.cpp
)

target_compile_features(fanuc PUBLIC cxx_std_17)
target_include_directories(fanuc PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/fanuc>
)
ament_target_dependencies(
  fanuc 
  ${HW_IF_INCLUDE_DEPENDS}
)

target_link_libraries(rmi_communication   ${rclcpp_LIBRARIES})
target_link_libraries(rmi_driver          rmi_communication )  
target_link_libraries(fanuc_eth_ip        EIPScanner)
target_link_libraries(fanuc               fanuc_eth_ip rmi_driver)  

ament_target_dependencies(rmi_communication rclcpp)


pluginlib_export_plugin_description_file(hardware_interface hw_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/fanuc
)
install(
  DIRECTORY include/
  DESTINATION include/fanuc_eth_ip
)
install(
  DIRECTORY include/
  DESTINATION include/rmi_communication
)
install(
  DIRECTORY include/
  DESTINATION include/rmi_driver
)

install(TARGETS fanuc
  EXPORT fanuc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS fanuc_eth_ip
  EXPORT fanuc_eth_ip
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rmi_communication
  EXPORT rmi_communication
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rmi_driver
  EXPORT rmi_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(fanuc_eth_ip HAS_LIBRARY_TARGET)
ament_export_targets(fanuc HAS_LIBRARY_TARGET)
ament_export_targets(rmi_communication HAS_LIBRARY_TARGET)
ament_export_targets(rmi_driver HAS_LIBRARY_TARGET)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(dpm src/dpm.cpp)
target_link_libraries(dpm fanuc_eth_ip)

ament_target_dependencies(
  dpm 
  ${HW_IF_INCLUDE_DEPENDS}
  std_srvs
)
install(TARGETS
dpm
DESTINATION lib/${PROJECT_NAME})


add_executable(rmi src/rmi.cpp)
target_link_libraries(rmi rmi_driver)

ament_target_dependencies(
  rmi
  ${HW_IF_INCLUDE_DEPENDS}
  )
install(TARGETS
rmi
DESTINATION lib/${PROJECT_NAME})



ament_package()
