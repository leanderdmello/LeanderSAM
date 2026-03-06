// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "control_command_server.hpp"
#include "control_command_handler.hpp"
#include "configuration.hpp"

// C++ Standard Library
#include <thread>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("control_command_node");
    executor.add_node(node);

    Configuration config(node);
    auto          command_server = std::make_shared<ControlCommandServer>(node, config.port_);
    std::thread   websocket_thread(&ControlCommandServer::Run, command_server.get());

    auto command_handler = std::make_unique<ControlCommandHandler>(command_server, node, config);
    std::thread command_thread(&ControlCommandHandler::Run, command_handler.get());

    executor.spin();

    command_server->Stop();
    websocket_thread.join();
    command_thread.join();
    rclcpp::shutdown();
    return 0;
}
