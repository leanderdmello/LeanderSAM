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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# fetch and compile GL resources like glfw glm and glad from source
# not a guaranteed that the jetson orin provides these and are up-to-date
include(${CMAKE_CURRENT_LIST_DIR}/openGL.cmake)

# stb image loading for textures and sprites
include(${CMAKE_CURRENT_LIST_DIR}/stb.cmake)

#freetype for text rendering
include(${CMAKE_CURRENT_LIST_DIR}/freetype.cmake)

# Tracy profiler for performance analysis
include(${CMAKE_CURRENT_LIST_DIR}/tracy.cmake)

set(pc_sources
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/camera.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/camera.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/floor_indicator.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/floor_indicator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/hud_element.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/hud_element.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/marker.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/marker.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/point_cloud_shaders.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/point_cloud.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/point_cloud.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/quad_shaders.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/renderer.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/renderer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/robot.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/robot.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/scale_indicator.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/scale_indicator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/shader.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/stb_wrapper.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/stb_wrapper.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/texture.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/texture.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/text_renderer.h
    ${CMAKE_CURRENT_LIST_DIR}/pc_renderer/text_renderer.cpp
)

set(node_sources
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/map_visualizer.h
    ${CMAKE_CURRENT_LIST_DIR}/map_visualizer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../helpers/shared_memory.h
)

add_executable(map_visualizer
    ${node_sources}
    ${pc_sources}
)

target_link_libraries(map_visualizer OpenGL::OpenGL OpenGL::EGL glm gbm glad stb freetype TracyClient)

install(TARGETS map_visualizer
    DESTINATION lib/${PROJECT_NAME})

include(GNUInstallDirs)
install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
# runtime resources
install(DIRECTORY resources/ DESTINATION share/${PROJECT_NAME}/resources)

# ros specific
ament_target_dependencies(map_visualizer
    rclcpp
    std_msgs
    interface_msgs
    std_srvs
    geometry_msgs
    nav_msgs)

ament_package()
create_debian_package(PACKAGE_NAME_PREFIX telerob)

# need to have nvidia-container-toolkit

# to force opengl to discrete - all you need: (in terminal)

# export __NV_PRIME_RENDER_OFFLOAD=1
# export __GLX_VENDOR_LIBRARY_NAME=nvidia

# add --gpus=all --runtime nvidia to docker run
# This will make gpu available inside the container.
# You also need to set runtime nvidia, else you will get CPU implementation :)

# Like our living legend Linus said:
#
#     F.U NVIDIA
#
