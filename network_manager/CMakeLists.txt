cmake_minimum_required(VERSION 3.16)

find_package(cmake_avular REQUIRED)
parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(
  ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  DESCRIPTION ${PROJECT_DESCRIPTION}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_msgs REQUIRED)

add_executable(network_manager src/network_manager.cpp)
target_include_directories(network_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(network_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS network_manager
  DESTINATION lib/${PROJECT_NAME})

include(GNUInstallDirs)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

ament_target_dependencies(network_manager
  rclcpp
  interface_msgs
)

add_linting_tests()

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
