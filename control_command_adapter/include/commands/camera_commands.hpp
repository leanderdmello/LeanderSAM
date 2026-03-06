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
#include "std_srvs/srv/trigger.hpp"

#include "interface_msgs/srv/camera_movement.hpp"
#include "interface_msgs/srv/follow_robot.hpp"

#include "configuration.hpp"

class CameraCommands
{
public:
    CameraCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // Camera commands
    void           ModifyCamera(const std::optional<nlohmann::json> &params);
    nlohmann::json MoveCameraToRobotLocation();
    void           SetFollowRobotOption(const nlohmann::json &params);
    nlohmann::json SetRobotLocationToCameraPosition();

private:
    // ROS service clients
    // Camera commands
    rclcpp::Client<interface_msgs::srv::CameraMovement>::SharedPtr modify_camera_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr move_camera_to_robot_location_client_;
    rclcpp::Client<interface_msgs::srv::FollowRobot>::SharedPtr set_follow_robot_option_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_robot_location_to_camera_position_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
