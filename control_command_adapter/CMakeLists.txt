cmake_minimum_required(VERSION 3.16)

find_package(cmake_avular REQUIRED)
parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(
  ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  DESCRIPTION ${PROJECT_DESCRIPTION}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(websocketpp REQUIRED)

add_executable(control_command_adapter
  src/control_command_adapter.cpp
  src/control_command_server.cpp
  src/control_command_handler.cpp
  src/commands/floor_commands.cpp
  src/commands/marker_commands.cpp
  src/commands/map_commands.cpp
  src/commands/camera_commands.cpp
  src/commands/misc_commands.cpp
  src/commands/stream_commands.cpp
  src/configuration.cpp)

target_compile_features(control_command_adapter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
set(CMAKE_CXX_STANDARD 17)

target_include_directories(control_command_adapter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(control_command_adapter
  websocketpp::websocketpp
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(control_command_adapter
  rclcpp interface_msgs std_srvs)

add_linting_tests()

install(TARGETS control_command_adapter
  DESTINATION lib/${PROJECT_NAME})

include(GNUInstallDirs)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_floor_commands.py)
  add_ros_isolated_launch_test(test/test_camera_commands.py)
  add_ros_isolated_launch_test(test/test_map_commands.py)
  add_ros_isolated_launch_test(test/test_misc_commands.py)
  add_ros_isolated_launch_test(test/test_marker_commands.py)
  add_ros_isolated_launch_test(test/test_stream_commands.py)
  add_ros_isolated_launch_test(test/test_malformed_commands.py)
endif()

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
