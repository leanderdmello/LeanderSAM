cmake_minimum_required(VERSION 3.16)

find_package(cmake_avular REQUIRED)
parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(
  ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  DESCRIPTION ${PROJECT_DESCRIPTION}
)

cxx_settings_exec()

# When adding a dependency, you will have to both add it to the package.xml and
# this file. The dependencies listed in the package.xml are used to determine
# what other packages have to be installed or built first. Inside the
# CMakeLists.txt, the dependencies are used to find the packages and link the
# libraries. This is done in three steps: 1. find_package() to find the package
# 2. target_link_libraries() to link the libraries 3.
# ament_export_dependencies() to avoid downstream packages to have to
# find_package for the dependencies in this package
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp_avular REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(slam_manager_msgs REQUIRED)

# We create a separate library for the node, which we do not make available
# outside of this package This allows us to link against it for testing
# purposes, and keeps the door open for potential future uses as a plugin.
add_library(
  ${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp
)

# Here we link against our dependencies. It is preferred to link using modern
# cmake: `modern_cmake_lib::modern_cmake_lib` However, not all packages support
# this, some notable exceptions: * ROS2 msgs packages have to be linked as
# `<package_name>_TARGETS` * Nav2 packages have to be linked as
# `<package_name>_LIBRARIES`. This also goes for other older ROS2 packages.
target_link_libraries(
  ${PROJECT_NAME}_node
  PUBLIC ${autonomy_msgs_TARGETS}
         ${slam_manager_msgs_TARGETS}
         fmt::fmt
         rclcpp_avular::rclcpp_avular
)

# Actual executable is just the main function
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_node)


# Tests
include(CTest)
# add_linting_tests()

if(BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
  )
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE
  )
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_subdirectory(test)
endif()

# Install
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# Here we export the packages for which we did find_package and linked against in our package
# This avoids the need for downstream packages to have to call find_package in order to link
# against these dependencies.
ament_export_dependencies(
  autonomy_msgs
  slam_manager_msgs
  fmt
  rclcpp_avular
)

# Package
ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
