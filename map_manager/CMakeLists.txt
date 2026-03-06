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
find_package(slam_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs  REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(nav_msgs  REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(map_manager src/map_manager.cpp src/map_manager_node.cpp src/slam_data_handler.cpp src/states/base_state.cpp src/states/idle_state.cpp src/states/mapping_state.cpp src/states/localizing_state.cpp src/pointcloud_file_helper.cpp src/pointcloud_shm_helper.cpp src/configuration.cpp)

target_compile_features(map_manager PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
set(CMAKE_CXX_STANDARD 20)

target_include_directories(map_manager PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/states)

target_link_libraries(map_manager
)

ament_target_dependencies(map_manager
  rclcpp interface_msgs slam_manager_msgs std_srvs sensor_msgs nav_msgs pcl_conversions PCL tf2 tf2_ros tf2_geometry_msgs)

add_linting_tests()

install(TARGETS map_manager
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
endif()

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
