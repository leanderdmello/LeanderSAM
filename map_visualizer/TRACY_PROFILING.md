# Tracy Profiler Integration

This document describes the Tracy profiler integration in the map_visualizer node.

## Version Information

- **Tracy Library Version**: v0.12.1
- **Profiler GUI Version**: v0.12.1 (must match library version)

## Overview

Tracy is a real-time, nanosecond resolution, remote telemetry profiler for games and other applications. It has been integrated into the map_visualizer to help diagnose performance issues, particularly with map data processing and rendering.

## What's Instrumented

The following areas are instrumented with Tracy:

### Async Data Handlers
- **ReadPointCloudData**: Tracks incoming point cloud data from SLAM
- **ReadTrajectoryData**: Tracks trajectory updates
- **ProcessLocalizationPath**: Tracks localization path updates

These handlers use Tracy's fiber support (`TracyFiberEnter`/`TracyFiberLeave`) to properly track async operations across thread boundaries.

### Rendering Operations
- **RenderLoop**: Main render thread with frame markers
- **RenderLoop::Iteration**: Individual render loop iterations
- **ProcessRenderCommand**: Command queue processing
- **RenderFrame**: Frame rendering (when frame time threshold is met)
- **Map_Renderer::Render**: Core rendering function

### GPU Operations
- **UploadToGPU**: Point cloud data upload to GPU
- **GPU::UploadPointCloud**: GPU zone for upload operations
- **GPU::RenderFrame**: GPU zone for rendering operations

### Preprocessing Operations
- **PreprocessMap::File**: Loading and preprocessing map from file
- **PreprocessMap::Memory**: Loading and preprocessing map from memory
- **PreprocessTrajectory::File**: Loading and preprocessing trajectory from file
- **PreprocessTrajectory::Memory**: Loading and preprocessing trajectory from memory

## Tracy Configuration

Tracy is configured with the following options (see `tracy.cmake`):

- `TRACY_ENABLE=ON`: Enable profiling
- `TRACY_ON_DEMAND=ON`: Only profile when Tracy client is connected
- `TRACY_NO_EXIT=ON`: Disable exit hooks (important for ROS nodes)
- `TRACY_FIBERS=ON`: Enable fiber support for async operations

### Headers Required

- `<tracy/Tracy.hpp>`: Main Tracy header for CPU profiling
- `<tracy/TracyOpenGL.hpp>`: Required for GPU profiling with OpenGL (must be included after Tracy.hpp)

### Important Notes

- Thread naming uses `tracy::SetThreadName()` function (C++ API)
- GPU profiling macros (`TracyGpuContext`, `TracyGpuZone`, `TracyGpuCollect`) require `TracyOpenGL.hpp`

## Using Tracy

### 1. Build the map_visualizer with Tracy

The Tracy profiler is automatically included when building map_visualizer:

```bash
colcon build --packages-select map_visualizer
```

### 2. Run the Tracy Profiler GUI

Download and run the Tracy profiler from: https://github.com/wolfpld/tracy/releases

**Important**: Make sure to use Tracy profiler version 0.12.1 or compatible to match the instrumented code version.

On Linux:
```bash
# Download Tracy profiler v0.12.1
wget https://github.com/wolfpld/tracy/releases/download/v0.12.1/Tracy-0.12.1.tar.gz
tar -xzf Tracy-0.12.1.tar.gz
cd Tracy-0.12.1/profiler/build/unix
make release
./Tracy-release
```

### 3. Launch map_visualizer

Run the map_visualizer node as usual:

```bash
ros2 run map_visualizer map_visualizer
```

### 4. Connect Tracy

With both the Tracy GUI and map_visualizer running:
1. The map_visualizer will appear in the Tracy client's connection list
2. Click "Connect" to start profiling
3. Tracy will begin collecting performance data

**Version Compatibility Note**: The Tracy GUI version must match the library version (v0.12.1). Mismatched versions may result in connection failures or incorrect profiling data. If you experience issues, ensure both the instrumented code and profiler GUI are using v0.12.1.

## Analyzing Performance

### Key Metrics to Monitor

1. **Frame Time**: Look at the `RenderLoop::Iteration` zones to see frame-to-frame timing
2. **Data Handler Blocking**: Check `ReadPointCloudData` duration - this should be quick
3. **GPU Upload Time**: Monitor `UploadToGPU` - spikes here indicate large data transfers
4. **Preprocessing Time**: Check `PreprocessMap::Memory` - CPU-intensive work happens here
5. **Render Command Queue**: Look at `ProcessRenderCommand` frequency and duration

### Common Issues

- **Rendering blocked by data**: If `ReadPointCloudData` shows long duration, map data is blocking
- **GPU transfer bottleneck**: Long `UploadToGPU` times indicate GPU memory bandwidth issues
- **Frame drops**: Check if render commands are accumulating in the queue

## Plots

Tracy plots are available for:
- `MapDataSize`: Size of incoming map data in bytes
- `TrajectoryDataSize`: Size of trajectory data in bytes
- `PointCloudSize`: Number of points in the point cloud

## GPU Profiling

GPU profiling is enabled but note that:
- The map_visualizer uses EGL and renders to an offscreen buffer
- GPU timing queries should still work, but there's no display synchronization
- GPU zones: `GPU::UploadPointCloud` and `GPU::RenderFrame`

## Disabling Tracy

To build without Tracy profiling, comment out the tracy.cmake include in CMakeLists.txt:

```cmake
# Tracy profiler for performance analysis
# include(${CMAKE_CURRENT_LIST_DIR}/tracy.cmake)
```

And remove TracyClient from the link libraries:

```cmake
target_link_libraries(map_visualizer OpenGL::OpenGL OpenGL::EGL glm gbm glad stb freetype) # TracyClient removed
```

## References

- Tracy Profiler: https://github.com/wolfpld/tracy
- Tracy Manual: https://github.com/wolfpld/tracy/releases/latest/download/tracy.pdf
