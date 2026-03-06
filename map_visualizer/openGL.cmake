set(OpenGL_GL_PREFERENCE "GLVND")
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)

IF (OPENGL_FOUND)
    message("Found OpenGL on this system")
    message(${OPENGL_LIBRARIES})
    else()
    message(FATAL_ERROR "CRITICAL DEPENDENCY WARNING! OpenGL is not installed on your system.")
endif()

message(${OPENGL_egl_LIBRARY})
message(${OPENGL_EGL_INCLUDE_DIRS})

include(FetchContent)

# GLAD OpenGL 4.3 core or newer
# glad is configured and generated for specific tagets
include(${CMAKE_CURRENT_LIST_DIR}/glad/glad.cmake)

#glfw dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(GBM QUIET gbm)

if (NOT GBM_FOUND)
    message(FATAL_ERROR "Missing dependency: libgbm-dev (pkg-config module 'gbm')")
endif()

pkg_check_modules(GLU QUIET glu)

if (NOT GLU_FOUND)
    message(FATAL_ERROR "Missing dependency: libgl1-mesa-dev (pkg-config module 'glu')")
endif()

pkg_check_modules(EGL QUIET egl)

if (NOT EGL_FOUND)
    message(FATAL_ERROR "Missing dependency: libegl1-mesa-dev (pkg-config module 'egl')")
endif()

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
FetchContent_MakeAvailable(glm)
