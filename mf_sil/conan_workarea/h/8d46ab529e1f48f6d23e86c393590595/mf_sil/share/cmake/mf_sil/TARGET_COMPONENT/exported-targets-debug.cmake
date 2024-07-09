#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mf_sil::ap_wrapper" for configuration "Debug"
set_property(TARGET mf_sil::ap_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(mf_sil::ap_wrapper PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/ap_wrapper.lib"
  )

list(APPEND _cmake_import_check_targets mf_sil::ap_wrapper )
list(APPEND _cmake_import_check_files_for_mf_sil::ap_wrapper "${_IMPORT_PREFIX}/lib/ap_wrapper.lib" )

# Import target "mf_sil::CarMaker.win64" for configuration "Debug"
set_property(TARGET mf_sil::CarMaker.win64 APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(mf_sil::CarMaker.win64 PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/CarMaker.win64.exe"
  )

list(APPEND _cmake_import_check_targets mf_sil::CarMaker.win64 )
list(APPEND _cmake_import_check_files_for_mf_sil::CarMaker.win64 "${_IMPORT_PREFIX}/bin/CarMaker.win64.exe" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
