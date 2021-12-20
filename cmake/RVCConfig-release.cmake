#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "RVC" for configuration "Release"
set_property(TARGET RVC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(RVC PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libRVC.so.1.3.0"
  IMPORTED_SONAME_RELEASE "libRVC.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS RVC )
list(APPEND _IMPORT_CHECK_FILES_FOR_RVC "${_IMPORT_PREFIX}/lib/libRVC.so.1.3.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
