

#Define component folder path
SET(comp_folder "../")
SET(include_folder "tests/SIL/CarMaker/include/CopiedIncludes")
SET(build_folder "../../build")

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Copy DFI and CML artefacts to local folders
find_package(cml REQUIRED)
find_package(adas_platforms REQUIRED)
find_package(MOCOLI REQUIRED)

# Don't build the carmaker solution automatically
# As the correct dependency tracking is not working.
# A manual build with --target carmaker is requried.
#set_target_properties(carmaker PROPERTIES EXCLUDE_FROM_ALL True)

# find components
MACRO(ADD_MOCO_COMPONENT COMP_NAME VARIANT)
  IF (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${comp_folder}/${COMP_NAME})
    MESSAGE(STATUS "${COMP_NAME} source directory found.")
    SET(${COMP_NAME}_PROJECT ${VARIANT} CACHE STRING "" FORCE)
    SET(${COMP_NAME}_PLATFORM vsp CACHE STRING "" FORCE)
    
    add_subdirectory(${comp_folder}/${COMP_NAME} ${build_folder}/${COMP_NAME})

    SET_TARGET_PROPERTIES(${COMP_NAME} PROPERTIES FOLDER DF/${COMP_NAME})
    SET_TARGET_PROPERTIES(${COMP_NAME}_vsp PROPERTIES FOLDER DF/${COMP_NAME})
    
    TARGET_INCLUDE_DIRECTORIES(${COMP_NAME}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${include_folder}>
    )
  ELSE()
    MESSAGE(STATUS "${COMP_NAME} source directory does not exist at ${comp_folder}/${COMP_NAME}")
  ENDIF()
ENDMACRO()


ADD_MOCO_COMPONENT(TRATCO PLP)
ADD_MOCO_COMPONENT(VECONA PLP)


SOURCE_GROUP(TREE ${comp_folder} PREFIX DF)

