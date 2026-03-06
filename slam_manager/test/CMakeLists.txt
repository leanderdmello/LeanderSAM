include(GoogleTest)

# Source files for unit tests.
set(test_files main.cpp)

add_executable(${PROJECT_NAME}_tests ${test_files})

target_link_libraries(${PROJECT_NAME}_tests gtest ${PROJECT_NAME}_node)
target_include_directories(${PROJECT_NAME}_tests PRIVATE ../src)
ament_target_dependencies(${PROJECT_NAME}_tests rclcpp)

gtest_discover_tests(${PROJECT_NAME}_tests)
