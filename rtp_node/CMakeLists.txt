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
find_package(std_msgs REQUIRED)
find_package(interface_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# cmake gstreamer module
include(${CMAKE_CURRENT_LIST_DIR}/gstreamer.cmake)

add_executable(rtp_node
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/rtp_stream.h
    ${CMAKE_CURRENT_LIST_DIR}/rtp_stream.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../helpers/shared_memory.h)

target_link_libraries(rtp_node gstreamer)

add_copyright_test()

install(TARGETS rtp_node
    DESTINATION lib/${PROJECT_NAME})

include(GNUInstallDirs)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

# ros specific
ament_target_dependencies(rtp_node
  rclcpp
  std_msgs
  interface_msgs
  std_srvs)

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
