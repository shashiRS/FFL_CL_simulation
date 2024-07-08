cmake_minimum_required (VERSION 3.1)

set(CMAKE_CONFIGURATION_TYPES Debug Debug_Cus Release CACHE TYPE INTERNAL FORCE)

project(ParkingVisu)

set(CMAKE_C_FLAGS_DEBUG_CUS "${CMAKE_C_FLAGS_DEBUG}" CACHE STRING "Flags used by the C compiler during Debug_Cus builds." FORCE)
set(CMAKE_CXX_FLAGS_DEBUG_CUS "${CMAKE_CXX_FLAGS_DEBUG}" CACHE STRING "Flags used by the CXX compiler during Debug_Cus builds." FORCE)
set(CMAKE_EXE_LINKER_FLAGS_DEBUG_CUS "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" CACHE STRING "CMAKE_EXE_LINKER_FLAGS_Debug_Cus." FORCE)
set(CMAKE_MODULE_LINKER_FLAGS_DEBUG_CUS "${CMAKE_MODULE_LINKER_FLAGS_DEBUG}" CACHE STRING "CMAKE_MODULE_LINKER_FLAGS_Debug_Cus." FORCE)

# suppress C4067 ("warning C4067: unexpected tokens following preprocessor directive - expected a newline")
# suppress C4267 ("'return': conversion from 'size_t' to 'lsm_geoml::size_type', possible loss of data")
# suppress compiler warning C4996 (deprecated warning), since the Cobolt-generated enums result in a lot of deprecation warnings
add_compile_options(/wd4067)
add_compile_options(/wd4267)
add_compile_options(/wd4996)

# set cus_only preprocessor definitions
set(CUS_PREDEF
    ULTRASONIC_ONLY
)

add_subdirectory(core)
add_subdirectory(visuclientinterface)
add_subdirectory(visuclientlib)
add_subdirectory(visulib)
add_subdirectory(app)
