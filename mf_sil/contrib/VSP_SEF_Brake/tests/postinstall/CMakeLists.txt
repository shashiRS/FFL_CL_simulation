
cmake_minimum_required(VERSION 3.5)
project( SmokeTest_PostInstall_Project )


# Only info.
install(
    CODE  "MESSAGE(\"*** SmokeTest_PostInstall_Project***\")" # argument of CODE must be a single line
    COMPONENT TARGET_COMPONENT
)

# Do the Smoke Test via "SCRIPT".
install(
     SCRIPT "${CMAKE_SOURCE_DIR}/tests/postinstall/PostInstall.cmake"
     COMPONENT TARGET_COMPONENT
)
