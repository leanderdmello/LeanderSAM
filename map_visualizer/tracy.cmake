# Tracy Profiler integration via FetchContent
include(FetchContent)

# Set Tracy options before fetching
set(TRACY_ENABLE ON CACHE BOOL "Enable Tracy profiler" FORCE)
set(TRACY_ON_DEMAND ON CACHE BOOL "Enable on-demand profiling" FORCE)
set(TRACY_NO_EXIT OFF CACHE BOOL "Disable exit hooks" FORCE)
set(TRACY_FIBERS ON CACHE BOOL "Enable fiber support for async operations" FORCE)

# Fetch Tracy from GitHub
FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.12.1
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(tracy)

# TracyClient is the main library to link against
# Note: For GPU profiling, include <tracy/TracyOpenGL.hpp> in files that use GPU zones
