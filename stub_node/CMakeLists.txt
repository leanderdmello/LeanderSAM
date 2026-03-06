cmake_minimum_required(VERSION 3.16)
find_package(cmake_avular REQUIRED)

project(stub_node VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(cmake_avular REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_msgs REQUIRED)
find_package(slam_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(stub_node
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
)

add_linting_tests()

install(TARGETS stub_node
    DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(stub_node
  rclcpp interface_msgs std_srvs slam_manager_msgs)

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
