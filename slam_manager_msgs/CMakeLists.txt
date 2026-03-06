cmake_minimum_required(VERSION 3.16)

find_package(cmake_avular REQUIRED)
parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(
  ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  DESCRIPTION ${PROJECT_DESCRIPTION}
)

# When adding a dependency, you will have to both add it to the package.xml and
# this file.
# The dependencies listed in the package.xml are used to determine what other interface packages
# have to be installed or built first.
# Inside the CMakeLists.txt, the dependencies are used to find the packages and link the
# interfaces. This is done in three steps:
# 1. find_package() to find the package
# 2. rosidl_generate_interfaces() to link the interface packages
# 3. ament_export_dependencies() to avoid downstream packages to have to call
#    `find_package` for the dependencies of this package
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSGS
    # cmake-format: sortable
    msg/SlamMode.msg
)

set(SRVS
    # cmake-format: sortable
    srv/SetSlamMode.srv
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${SRVS}
  ${MSGS}
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

# Package
ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
