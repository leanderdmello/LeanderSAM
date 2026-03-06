# GLAD is a Vulkan/GL/GLES/EGL/GLX/WGL Loader-Generator based on
# the official specifications for multiple languages.
#
# Glad can be ran from source: 
#   https://github.com/Dav1dde/glad?tab=readme-ov-file
# or via the GLAD web page: 
#   https://glad.dav1d.de/  (version 1)
#   https://gen.glad.sh/    (version 2)
#
# This implementation is based on Version 1, with openGL support up to 4.6,
# The version Jetson nano orin supports. 
# 
# OpenGL is only really a standard/specification it is up to the driver manufacturer
# to implement the specification to a driver that the specific graphics card supports.
# Since there are many different versions of OpenGL drivers,
# the location of most of its functions is not known at compile-time
# and needs to be queried at run-time. It is then the task of the developer
# to retrieve the location of the functions he/she needs and store them in
# function pointers for later use. This looks like:
#
# // define the function's prototype
# typedef void (*GL_GENBUFFERS) (GLsizei, GLuint*);
# // find the function and assign it to a function pointer
# GL_GENBUFFERS glGenBuffers  = (GL_GENBUFFERS)wglGetProcAddress("glGenBuffers");
# // function can now be called as normal
# unsigned int buffer;
# glGenBuffers(1, &buffer);
# 
# As you can see the code looks complex and it's a cumbersome process to do this
# for each function you may need that is not yet declared. Thankfully, there are
# libraries for this purpose as well where GLAD is a popular and up-to-date library. 

add_library(glad "")

target_sources(glad
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/glad.c
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include/glad/glad.h
        ${CMAKE_CURRENT_LIST_DIR}/include/KHR/khrplatform.h
)

target_include_directories(glad PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
