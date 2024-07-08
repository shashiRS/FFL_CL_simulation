
cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)

project(mf_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic-errors -Werror")

add_definitions(-DAP_USE_NAMESPACES)
add_definitions(-DAP_USE_ENUM_CLASSES)

add_subdirectory(src/component)

# -- documentation
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
    doxygen_add_docs(DOCUMENTATION
                     doc
                     src/component
                    )

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
            COMPONENT DOCUMENTATION
            DESTINATION doc/${PROJECT_NAME}
           )
endif(Doxygen_FOUND)



