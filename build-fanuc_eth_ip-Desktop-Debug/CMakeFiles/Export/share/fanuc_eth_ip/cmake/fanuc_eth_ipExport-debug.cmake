#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fanuc_eth_ip::fanuc_eth_ip" for configuration "Debug"
set_property(TARGET fanuc_eth_ip::fanuc_eth_ip APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(fanuc_eth_ip::fanuc_eth_ip PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libfanuc_eth_ip.so"
  IMPORTED_SONAME_DEBUG "libfanuc_eth_ip.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS fanuc_eth_ip::fanuc_eth_ip )
list(APPEND _IMPORT_CHECK_FILES_FOR_fanuc_eth_ip::fanuc_eth_ip "${_IMPORT_PREFIX}/lib/libfanuc_eth_ip.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
