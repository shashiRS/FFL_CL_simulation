# group source files for IDE generation
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR})

find_package(us_drv_types REQUIRED)
find_package(appl_srv_layer_types REQUIRED)
find_package(carmaker_build_dep) # not REQUIRED: only for vs2017 & vs2017_debug

#----------- CarMaker-related settings ----------------
if(CMAKE_HOST_WIN32)
    set(ARCH win64)
elseif(UNIX)
    set(ARCH linux64)
endif()

# related version of CarMaker application
set(CARMAKER_VER 8.1.1)
set(CARMAKER_NUMVER 80101)
# directories from CarMaker build dependency package
set(CARMAKER_BIN_DIR ${carmaker_build_dep_PACKAGE_PATH}/bin)
set(CARMAKER_LIB_DIR ${carmaker_build_dep_PACKAGE_PATH}/lib)
set(CARMAKER_INC_DIR ${carmaker_build_dep_PACKAGE_PATH}/include)

#----------- vcus_phen_mod library ----------------
set(TARGET_NAME vcus_phen_mod)

set(THIS_SRC
    Init_Clean.cpp
    Reflection_Generator.cpp
    ReflectionFunc.cpp
    Sensor_Mapping.cpp
    vCUS_Main.cpp
    vCUS_Utils.cpp
)

add_library(${TARGET_NAME} STATIC
    ${THIS_SRC}
)

target_include_directories(${TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CARMAKER_INC_DIR}>
)

target_link_libraries(${TARGET_NAME}
  # type libraries made public for users of this library
  PUBLIC
    us_drv_types::types
    appl_srv_layer_types::appl_srv_layer_types
  PRIVATE
    ${CARMAKER_LIB_DIR}/libcarmaker.a
    ${CARMAKER_LIB_DIR}/libcar.a
    ${CARMAKER_LIB_DIR}/libipgdriver.a
    ${CARMAKER_LIB_DIR}/libipgroad.a
    ${CARMAKER_LIB_DIR}/libipgtire.a
    ${CARMAKER_LIB_DIR}/libz-${ARCH}.a
)

if(WIN32)
    target_link_libraries(${TARGET_NAME}
      PRIVATE
        ${CARMAKER_LIB_DIR}/libcompat_${ARCH}.a
        ${CARMAKER_LIB_DIR}/libcompat_msvs2015.a
        user32.lib
        advapi32.lib
        ws2_32.lib
        legacy_stdio_definitions.lib
        winspool.lib
    )

    target_compile_definitions(${TARGET_NAME}
      PRIVATE
        WIN64
        CM_NUMVER=${CARMAKER_NUMVER}
    )
endif()

install(
  TARGETS ${TARGET_NAME}
  COMPONENT TARGET_COMPONENT
  DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  FILES
  PhenoModel.h
  vCUS_mfPlot.h
  us_drv_def_vfa.h
  COMPONENT TARGET_COMPONENT
  DESTINATION include/${TARGET_NAME}
)