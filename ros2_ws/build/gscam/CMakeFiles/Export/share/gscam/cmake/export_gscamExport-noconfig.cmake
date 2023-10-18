#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gscam::gscam" for configuration ""
set_property(TARGET gscam::gscam APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(gscam::gscam PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgscam.so"
  IMPORTED_SONAME_NOCONFIG "libgscam.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gscam::gscam )
list(APPEND _IMPORT_CHECK_FILES_FOR_gscam::gscam "${_IMPORT_PREFIX}/lib/libgscam.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
