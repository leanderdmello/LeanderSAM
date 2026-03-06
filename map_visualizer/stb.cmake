include(FetchContent)

# stb - single-header public domain libraries
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(stb)

# Create an INTERFACE target for stb headers
add_library(stb INTERFACE)

# stb is header-only, so just add include directories
target_include_directories(stb INTERFACE
    ${stb_SOURCE_DIR}
)
