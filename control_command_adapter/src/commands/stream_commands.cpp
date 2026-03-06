// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/stream_commands.hpp"

// C++ Standard Library

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"
#include "exceptions.hpp"

StreamCommands::StreamCommands(rclcpp::Node::SharedPtr node, const Configuration &config)
    : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;

    // Stream commands
    this->set_scale_unit_meters_client_ =
        this->node_->create_client<std_srvs::srv::SetBool>(config.scale_unit_meters_service_);
    this->set_video_resolution_mode_client_ =
        this->node_->create_client<interface_msgs::srv::SetVideoResolutionMode>(
            config.set_video_resolution_mode_service_);
    this->set_video_stream_address_client_ =
        this->node_->create_client<interface_msgs::srv::SetVideoStreamAddress>(
            config.set_video_stream_address_service_);
    this->set_video_stream_option_client_ = this->node_->create_client<std_srvs::srv::SetBool>(
        config.set_video_stream_enabled_service_);
    this->clear_path_client_ =
        this->node_->create_client<std_srvs::srv::Trigger>(config.clear_path_service_);
}

void StreamCommands::SetScaleUnit(const nlohmann::json &params)
{
    auto              request    = std::make_shared<std_srvs::srv::SetBool::Request>();
    const std::string scale_unit = params.at("ScaleUnit");
    if(scale_unit == "meters")
    {
        request->data = true;
    }
    else if(scale_unit == "feet")
    {
        request->data = false;
    }
    else
    {
        throw InvalidParameterValueException("ScaleUnit", "Expected values are: meters, feet");
    }
    RCLCPP_INFO(node_->get_logger(), "Set scale unit: %s.", scale_unit.c_str());
    call_service_no_wait<std_srvs::srv::SetBool>(this->set_scale_unit_meters_client_, request);
}

nlohmann::json StreamCommands::SetVideoResolutionMode(const nlohmann::json &params)
{
    auto request = std::make_shared<interface_msgs::srv::SetVideoResolutionMode::Request>();
    const std::string resolution_mode = params.at("ResolutionMode");
    if(resolution_mode == "highres")
    {
        request->mode = interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES;
    }
    else if(resolution_mode == "lowres")
    {
        request->mode = interface_msgs::srv::SetVideoResolutionMode::Request::LOWRES;
    }
    else
    {
        throw InvalidParameterValueException("ResolutionMode",
                                             "Expected values are: highres, lowres");
    }
    RCLCPP_INFO(node_->get_logger(), "Set video resolution mode: %s.", resolution_mode.c_str());
    return call_service_empty_response<interface_msgs::srv::SetVideoResolutionMode>(
        this->set_video_resolution_mode_client_, request, cmd_timeout_);
}

nlohmann::json StreamCommands::SetVideoStreamAddress(const nlohmann::json &params)
{
    auto request = std::make_shared<interface_msgs::srv::SetVideoStreamAddress::Request>();
    const std::string ip_address = params.at("IpAddress");
    const int         port       = params.at("Port");

    request->ip_address = ip_address;
    request->port       = port;
    RCLCPP_INFO(node_->get_logger(), "Set video stream adress: %s %d.", ip_address.c_str(), port);
    return call_service_empty_response<interface_msgs::srv::SetVideoStreamAddress>(
        this->set_video_stream_address_client_, request, cmd_timeout_);
}

nlohmann::json StreamCommands::SetVideoStreamOption(const nlohmann::json &params)
{
    auto       request             = std::make_shared<std_srvs::srv::SetBool::Request>();
    const bool video_stream_option = params.at("VideoStreamOption");
    RCLCPP_INFO(node_->get_logger(), "Set video stream option: %d.", video_stream_option);
    request->data = video_stream_option;
    return call_service_empty_response<std_srvs::srv::SetBool>(
        this->set_video_stream_option_client_, request, cmd_timeout_);
}

nlohmann::json StreamCommands::ClearPath()
{
    RCLCPP_INFO(node_->get_logger(), "Clear path.");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    return call_service_empty_response<std_srvs::srv::Trigger>(this->clear_path_client_, request,
                                                               cmd_timeout_);
}
