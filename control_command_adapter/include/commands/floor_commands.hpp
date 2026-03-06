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

#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/action_on_index.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/srv/get_index.hpp"
#include "interface_msgs/srv/update_string_at.hpp"

#include "configuration.hpp"

class FloorCommands
{
public:
    FloorCommands(rclcpp::Node::SharedPtr node, const Configuration &config);

    // JSON-RPC command handlersFloorCommands
    // Floor commands
    nlohmann::json AddFloor(const nlohmann::json &params);
    nlohmann::json DeleteFloor(const nlohmann::json &params);
    nlohmann::json ClearFloor();
    nlohmann::json GetFloors();
    nlohmann::json GetMappingFloor();
    nlohmann::json GetViewFloor();
    nlohmann::json SelectViewFloor(const nlohmann::json &params);
    nlohmann::json SelectMappingFloor(const nlohmann::json &params);
    nlohmann::json SetFloorName(const nlohmann::json &params);

private:
    // ROS service clients
    // Floor commands
    rclcpp::Client<interface_msgs::srv::SetString>::SharedPtr      add_floor_client_;
    rclcpp::Client<interface_msgs::srv::ActionOnIndex>::SharedPtr  delete_floor_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr              clear_floor_client_;
    rclcpp::Client<interface_msgs::srv::Listing>::SharedPtr        get_floors_client_;
    rclcpp::Client<interface_msgs::srv::GetIndex>::SharedPtr       get_mapping_floor_client_;
    rclcpp::Client<interface_msgs::srv::GetIndex>::SharedPtr       get_view_floor_client_;
    rclcpp::Client<interface_msgs::srv::ActionOnIndex>::SharedPtr  select_view_floor_client_;
    rclcpp::Client<interface_msgs::srv::ActionOnIndex>::SharedPtr  select_mapping_floor_client_;
    rclcpp::Client<interface_msgs::srv::UpdateStringAt>::SharedPtr set_floor_name_client_;

    rclcpp::Node::SharedPtr                node_;
    std::chrono::duration<int, std::milli> cmd_timeout_;
};
