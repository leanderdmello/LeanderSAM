// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/misc_commands.hpp"

// C++ Standard Library
#include <optional>
#include <sstream>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"
#include "exceptions.hpp"

MiscCommands::MiscCommands(rclcpp::Node::SharedPtr node, const Configuration &config) : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;

    // Misc commands
    this->report_robot_status_client_ =
        this->node_->create_publisher<interface_msgs::msg::RobotStatus>(config.robot_status_topic_,
                                                                        10);
    this->get_software_versions_client_ = this->node_->create_client<interface_msgs::srv::Listing>(
        config.get_software_versions_service_);
}

void MiscCommands::ReportRobotStatus(const nlohmann::json &params)
{
    bool                                    robot_is_moving           = params.at("RobotIsMoving");
    std::optional<std::vector<std::string>> optional_hardware_present = std::nullopt;
    if(params.contains("OptionalHardwarePresent"))
    {
        optional_hardware_present = params.at("OptionalHardwarePresent");
        std::stringstream stream;
        stream << "Report robot status moving: " << robot_is_moving << " optional params";
        for(const auto &option : optional_hardware_present.value())
        {
            stream << " " << option;
        }
        RCLCPP_INFO(node_->get_logger(), stream.str().c_str());
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Report robot status moving: %d.", robot_is_moving);
    }

    auto message            = interface_msgs::msg::RobotStatus();
    message.robot_is_moving = robot_is_moving;
    message.hardware_flags  = 0;
    if(optional_hardware_present)
    {
        for(const auto &hardware : optional_hardware_present.value())
        {
            if(hardware == "climbing_aid")
            {
                message.hardware_flags += interface_msgs::msg::RobotStatus::CLIMBING_AID;
            }
            else if(hardware == "long_flippers")
            {
                message.hardware_flags += interface_msgs::msg::RobotStatus::LONG_FLIPPERS;
            }
            else if(hardware == "short_flippers")
            {
                message.hardware_flags += interface_msgs::msg::RobotStatus::SHORT_FLIPPERS;
            }
            else
            {
                throw InvalidParameterValueException(
                    "OptionalHardware",
                    "Expected values are: climbing_aid, long_flippers, short_flippers");
            }
        }
    }
    this->report_robot_status_client_->publish(message);
}

nlohmann::json MiscCommands::GetSoftwareVersions()
{
    RCLCPP_INFO(node_->get_logger(), "Get software versions.");
    auto request       = std::make_shared<interface_msgs::srv::Listing::Request>();
    auto flat_response = call_service_with_list_response<interface_msgs::srv::Listing>(
        this->get_software_versions_client_, request, cmd_timeout_);

    // the response is a flat list of strings in the format "Component: Version"
    // we need to convert it to a JSON object with component names as keys and versions as values
    nlohmann::json response_json;
    for(const auto &entry : flat_response)
    {
        std::string entry_str     = entry.get<std::string>();
        auto        delimiter_pos = entry_str.find(':');
        if(delimiter_pos != std::string::npos)
        {
            std::string component = entry_str.substr(0, delimiter_pos);
            std::string version   = entry_str.substr(delimiter_pos + 1);
            // Trim whitespace
            component.erase(0, component.find_first_not_of(" \t"));
            component.erase(component.find_last_not_of(" \t") + 1);
            version.erase(0, version.find_first_not_of(" \t"));
            version.erase(version.find_last_not_of(" \t") + 1);
            response_json[component] = version;
        }
    }
    return response_json;
}
