#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pjsol::pjsol" for configuration ""
set_property(TARGET pjsol::pjsol APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pjsol::pjsol PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpjsol.so"
  IMPORTED_SONAME_NOCONFIG "libpjsol.so"
  )

list(APPEND _cmake_import_check_targets pjsol::pjsol )
list(APPEND _cmake_import_check_files_for_pjsol::pjsol "${_IMPORT_PREFIX}/lib/libpjsol.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
