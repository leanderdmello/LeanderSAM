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
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "interface_msgs/srv/set_video_resolution_mode.hpp"
#include "interface_msgs/srv/set_video_stream_address.hpp"

#include "configuration.hpp"

class StreamCommands
{
public:
    StreamCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // JSON-RPC command
    // Stream commands
    void           SetScaleUnit(const nlohmann::json &params);
    nlohmann::json SetVideoResolutionMode(const nlohmann::json &params);
    nlohmann::json SetVideoStreamAddress(const nlohmann::json &params);
    nlohmann::json SetVideoStreamOption(const nlohmann::json &params);
    nlohmann::json ClearPath();


private:
    // ROS service clients
    // Stream commands
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_scale_unit_meters_client_;
    rclcpp::Client<interface_msgs::srv::SetVideoResolutionMode>::SharedPtr
        set_video_resolution_mode_client_;
    rclcpp::Client<interface_msgs::srv::SetVideoStreamAddress>::SharedPtr
                                                      set_video_stream_address_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_video_stream_option_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_path_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
