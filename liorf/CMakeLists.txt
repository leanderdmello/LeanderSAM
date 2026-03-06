cmake_minimum_required(VERSION 3.16)
set(CMAKE_BUILD_TYPE "Release")

find_package(cmake_avular REQUIRED)
parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(
  ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  DESCRIPTION ${PROJECT_DESCRIPTION}
)

include(cmake/rosidl_get_typesupport_target.cmake)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_avular REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(interface_msgs REQUIRED)

set(ros2_lib "rclcpp;rclcpp_avular;autonomy_msgs;rclpy;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;cv_bridge;pcl_msgs;pcl_conversions;visualization_msgs;tf2;tf2_ros;tf2_eigen;tf2_sensor_msgs;tf2_geometry_msgs;interface_msgs")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# include directories
include_directories(
  lib
	include
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
  lib
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

###########
## Build ##
###########
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection_lifeCycle src/imageProjection_lifecycle.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imageProjection_lifeCycle ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imageProjection_lifeCycle "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# # Mapping Optimization LifeCycle
add_executable(${PROJECT_NAME}_mapOptmization_lifeCycle src/mapOptmization_lifecycle.cpp ${common_lib} include/Scancontext.cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization_lifeCycle PRIVATE ${OpenMP_CXX_FLAGS})
ament_target_dependencies(${PROJECT_NAME}_mapOptmization_lifeCycle ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_mapOptmization_lifeCycle Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam)

# # IMU Preintegration LifeCycle
add_executable(${PROJECT_NAME}_imuPreintegration_lifeCycle src/imuPreintegration_lifecycle.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration_lifeCycle ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration_lifeCycle Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
  ${PROJECT_NAME}_mapOptmization_lifeCycle
  ${PROJECT_NAME}_imageProjection_lifeCycle
  ${PROJECT_NAME}_imuPreintegration_lifeCycle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)

  # Tests
  include(CTest)
  add_linting_tests()

endif()

ament_export_include_directories(include)

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)
