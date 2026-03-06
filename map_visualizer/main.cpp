// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "map_visualizer.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto instance = std::make_shared<Map_Visualizer>();

    // Create dedicated render thread - OpenGL context will be bound to this thread
    std::thread render_thread([&instance]() { instance->RenderLoop(); });

    // Use MultiThreadedExecutor for ROS callbacks (data processing, services)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(instance);

    // Spin ROS executor on main thread
    executor.spin();

    // Cleanup
    instance->StopRenderLoop();
    if(render_thread.joinable())
    {
        render_thread.join();
    }

    instance->CleanupGLResources();
    rclcpp::shutdown();
    return 0;
}
