// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "rclcpp/rclcpp.hpp"
#include "map_manager_node.hpp"
#include "configuration.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto                                     node = std::make_shared<MapManagerNode>();
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
