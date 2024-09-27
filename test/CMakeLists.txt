find_package(GTest REQUIRED)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../transform3d
    ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE TEST_TOOLCOMPONENTS_SRC "*_test.cpp")

add_executable(test_tool_components ${TEST_TOOLCOMPONENTS_SRC})
target_link_libraries(test_tool_components
    ${LIB_TOOL_COMPONENT}
    ${GTEST_LIBRARIES}
)

add_test(
    NAME test_tool_components
    COMMAND test_tool_components
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)