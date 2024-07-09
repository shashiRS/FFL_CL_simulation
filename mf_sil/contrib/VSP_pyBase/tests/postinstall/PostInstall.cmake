
cmake_minimum_required(VERSION 3.5)

message (STATUS "*******************")
message (STATUS " PostInstall.cmake ")
message (STATUS "*******************")

# Debug
# Remark: When using the "cmake installing" not all of the usual variables exist.
message (STATUS "PROJECT_SOURCE_DIR:                    ${PROJECT_SOURCE_DIR} ")                    # empty!
message (STATUS "CMAKE_SOURCE_DIR:                      ${CMAKE_SOURCE_DIR} ")                      # D:/gf/VSP_SEF_Brake/conan_workarea/build.SEF_Brake.generic.1.0.0-fallback.vs201
message (STATUS "CMAKE_INSTALL_PREFIX:                  ${CMAKE_INSTALL_PREFIX} ")                  # D:/gf/VSP_SEF_Brake/conan_workarea/h/d1fb348771ab078ca3654950dcbdf012


# Replace "/" by "\".
file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX} CMAKE_INSTALL_PREFIX_WITH_BACKSLASHES)
message (STATUS "CMAKE_INSTALL_PREFIX_WITH_BACKSLASHES: ${CMAKE_INSTALL_PREFIX_WITH_BACKSLASHES} ")


# Debug
# execute_process(COMMAND ${CMAKE_COMMAND} -E echo "Do Smoke Test Postinstall " )

# Remark: Must be "/" here
# Typically: execute_process(COMMAND cmd /c D:/gf/VSP_SEF_Brake/scripts/start_autotest.bat D:\gf\VSP_SEF_Brake\conan_workarea\package.SEF_Brake.generic.1.0.0-fallback.vs2017  )
execute_process(COMMAND cmd /c ${CMAKE_SOURCE_DIR}/../../scripts/start_autotest.bat  ${CMAKE_INSTALL_PREFIX_WITH_BACKSLASHES}  )
