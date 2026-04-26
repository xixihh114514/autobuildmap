#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ZXing::Core" for configuration "None"
set_property(TARGET ZXing::Core APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(ZXing::Core PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libZXingCore.so.1.0.7"
  IMPORTED_SONAME_NONE "libZXingCore.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS ZXing::Core )
list(APPEND _IMPORT_CHECK_FILES_FOR_ZXing::Core "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libZXingCore.so.1.0.7" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
