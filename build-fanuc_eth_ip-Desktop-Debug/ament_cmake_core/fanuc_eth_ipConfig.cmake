# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fanuc_eth_ip_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fanuc_eth_ip_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fanuc_eth_ip_FOUND FALSE)
  elseif(NOT fanuc_eth_ip_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fanuc_eth_ip_FOUND FALSE)
  endif()
  return()
endif()
set(_fanuc_eth_ip_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fanuc_eth_ip_FIND_QUIETLY)
  message(STATUS "Found fanuc_eth_ip: 0.0.0 (${fanuc_eth_ip_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fanuc_eth_ip' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fanuc_eth_ip_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fanuc_eth_ip_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${fanuc_eth_ip_DIR}/${_extra}")
endforeach()
