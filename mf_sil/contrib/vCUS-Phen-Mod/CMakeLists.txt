cmake_minimum_required(VERSION 3.6)
project(vcus_phen_mod)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set compiler standard
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # using Visual Studio C++
    set (CMAKE_CXX_STANDARD 14)
endif()

add_subdirectory(vCUS_Phen_CM/src_PhenDevel/USS_PhenoModel)