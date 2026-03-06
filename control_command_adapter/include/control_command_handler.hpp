// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once
#include <memory>
#include <atomic>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "control_command_server.hpp"
#include "configuration.hpp"

#include "commands/floor_commands.hpp"
#include "commands/marker_commands.hpp"
#include "commands/map_commands.hpp"
#include "commands/camera_commands.hpp"
#include "commands/misc_commands.hpp"
#include "commands/stream_commands.hpp"

class ControlCommandHandler
{
public:
    ControlCommandHandler(std::shared_ptr<ControlCommandServer> server,
                          rclcpp::Node::SharedPtr               node,
                          Configuration                         config);
    void           Run();
    void           Stop();
    nlohmann::json HandleCommand(const std::string &json);

private:
    std::shared_ptr<ControlCommandServer> server_;

    rclcpp::Node::SharedPtr node_;
    FloorCommands           floor_commands_;
    MarkerCommands          marker_commands_;
    MapCommands             map_commands_;
    CameraCommands          camera_commands_;
    MiscCommands            misc_commands_;
    StreamCommands          stream_commands_;
};
