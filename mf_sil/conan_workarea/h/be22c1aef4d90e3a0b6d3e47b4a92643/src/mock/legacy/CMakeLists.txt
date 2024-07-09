add_library(${target_prefix}codegen_test INTERFACE)


list(REMOVE_DUPLICATES TEST_TYPES_PLANTUMLS)

target_include_directories(${target_prefix}codegen_test
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

get_filename_component(MOCK_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
set(MOCK_CALLBACK_DIR ${MOCK_DIR}/callback)
foreach(namespace IN LISTS namespaces)

  target_include_directories(${target_prefix}codegen_test
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${namespace}>
      $<BUILD_INTERFACE:${MOCK_CALLBACK_DIR}/${namespace}>
  )
endforeach()

target_link_libraries(${target_prefix}codegen_test
  INTERFACE
    eco::auxiliary
    ${target_prefix}codegen_mock_arguments
    ${target_prefix}codegen_mock_callback
    ${target_prefix}codegen_mock_debug
)
