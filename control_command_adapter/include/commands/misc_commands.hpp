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

#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/msg/robot_status.hpp"

#include "configuration.hpp"

class MiscCommands
{
public:
    MiscCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // JSON-RPC command
    // Misc commands
    void           ReportRobotStatus(const nlohmann::json &json);
    nlohmann::json GetSoftwareVersions();

private:
    // ROS service clients
    // Misc commands
    rclcpp::Publisher<interface_msgs::msg::RobotStatus>::SharedPtr report_robot_status_client_;
    rclcpp::Client<interface_msgs::srv::Listing>::SharedPtr        get_software_versions_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
