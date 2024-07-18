cmake_minimum_required(VERSION 3.8)
project(fanuc_rmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rcpputils REQUIRED)

include_directories(include)


set(HW_IF_INCLUDE_DEPENDS
  rcpputils
  rclcpp
  std_msgs
)

add_library(rmi_communication SHARED  src/rmi_communication.cpp)
add_library(rmi_driver SHARED  src/rmi_driver.cpp)

target_link_libraries(rmi_communication   ${rclcpp_LIBRARIES})
target_link_libraries(rmi_driver rmi_communication)


ament_target_dependencies(rmi_communication rclcpp)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS rmi_communication
  EXPORT rmi_communication
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
ament_export_libraries(rmi_communication)
ament_export_targets(rmi_communication HAS_LIBRARY_TARGET)

install(
  TARGETS rmi_driver
  EXPORT rmi_driver
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
ament_export_libraries(rmi_driver)
ament_export_targets(rmi_driver HAS_LIBRARY_TARGET)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


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
