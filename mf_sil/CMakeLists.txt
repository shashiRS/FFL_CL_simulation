cmake_minimum_required(VERSION 3.6)
project(mf_sil)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(Protobuf_USE_STATIC_LIBS ON)

if (${DEPENDENCY_EXPORT_ONLY}                           # do nothing, just export the dependencies via bricks
    OR
    (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")    #non-vs builds are for target -> do nothing
    OR 
    ${LOCAL_BUILD}                                      #local dependency build -> do nothing
    )
    
    set(DUMMY_NAME mf_sil_dummy)
    set(DUMMY_SRC
        src/component/mf_sil_dummy.cpp
    )
    add_library(${DUMMY_NAME} STATIC
        ${DUMMY_SRC}
    )
    target_link_libraries(${DUMMY_NAME}
      PUBLIC
      PRIVATE
    )
    install(
      TARGETS ${DUMMY_NAME}
      COMPONENT TARGET_COMPONENT
      DESTINATION lib
      INCLUDES DESTINATION include
    )
    
    if (${LOCAL_BUILD})
        include(scripts/localBuild.cmake)
    endif()

else()

    # set compiler standard
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        # using Visual Studio C++
        set (CMAKE_CXX_STANDARD 14)
        # suppress compiler warning C4996 (deprecated warning), since the Cobolt-generated enums result in a lot of deprecation warnings
        add_compile_options(/wd4996)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "TI")
        set(CMAKE_CXX_STANDARD 11)
        # ToDo: adapt flags for TI coompiler add_compile_options(-pedantic-errors -Werror -Wno-error=deprecated-declarations -Wno-error=missing-braces)
    else ()
        set (CMAKE_CXX_STANDARD 11)
        add_compile_options(-pedantic-errors -Werror -Wno-error=deprecated-declarations -std=c++11)
    endif()

    add_subdirectory(tests/SIL/CarMaker/src)
    # copy configuration files from used packages
    add_subdirectory(conf/package)

endif()

# -- documentation
if (NOT ${LOCAL_BUILD})
      
  find_package(Doxygen)
  if(Doxygen_FOUND)

      find_file(DOXYGEN_PLANTUML_JAR_PATH
                plantuml.jar
                "C:/cip_tools/plantuml/1.2019.3"
                "C:/LegacyApp/plantuml"
                "C:/Tools/plantuml"
                $ENV{PLANTUML_JAR_PATH}
                /usr/share/plantuml
               )
      doxygen_add_docs(DOCUMENTATION_${PROJECT_NAME}
                       #doc
                       tests/SIL/CarMaker/src
                      )
      if (NOT (TARGET DOCUMENTATION))
              add_custom_target(DOCUMENTATION COMMENT "workaround for multi-dependencies project")
              add_dependencies(DOCUMENTATION DOCUMENTATION_${PROJECT_NAME})
      endif()

      install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
              COMPONENT DOCUMENTATION
              DESTINATION doc/${PROJECT_NAME}
             )
  endif(Doxygen_FOUND)
endif()
