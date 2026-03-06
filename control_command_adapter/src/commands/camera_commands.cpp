// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "commands/camera_commands.hpp"

// C++ Standard Library
#include <sstream>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "service_helpers.hpp"
#include "exceptions.hpp"

CameraCommands::CameraCommands(rclcpp::Node::SharedPtr node, const Configuration &config)
    : node_(node)
{
    cmd_timeout_ = config.cmd_timeout_;
    // Camera commands
    this->modify_camera_client_ = this->node_->create_client<interface_msgs::srv::CameraMovement>(
        config.modify_camera_service_);
    this->move_camera_to_robot_location_client_ =
        this->node_->create_client<std_srvs::srv::Trigger>(config.move_camera_to_robot_service_);
    this->set_follow_robot_option_client_ =
        this->node_->create_client<interface_msgs::srv::FollowRobot>(config.follow_robot_service_);
    this->set_robot_location_to_camera_position_client_ =
        this->node_->create_client<std_srvs::srv::Trigger>(
            config.set_robot_location_to_camera_service_);
}

void CameraCommands::ModifyCamera(const std::optional<nlohmann::json> &params)
{
    std::stringstream stream;
    stream << "Modify camera";
    auto request = std::make_shared<interface_msgs::srv::CameraMovement::Request>();
    if(params)
    {
        if(params.value().contains("Zoom"))
        {
            request->update_zoom = true;
            request->dzoom       = params.value().at("Zoom");
            stream << " Zoom " << request->dzoom;
        }
        if(params.value().contains("XTranslation"))
        {
            request->update_x = true;
            request->dx       = params.value().at("XTranslation");
            stream << " XTranslation " << request->dx;
        }
        if(params.value().contains("YTranslation"))
        {
            request->update_y = true;
            request->dy       = params.value().at("YTranslation");
            stream << " YTranslation " << request->dy;
        }
        if(params.value().contains("Pan"))
        {
            request->update_pan = true;
            request->dpan       = params.value().at("Pan");
            stream << " Pan " << request->dpan;
        }
        if(params.value().contains("Tilt"))
        {
            request->update_tilt = true;
            request->dtilt       = params.value().at("Tilt");
            stream << " Tilt " << request->dtilt;
        }
    }
    RCLCPP_INFO(node_->get_logger(), stream.str().c_str());
    call_service_no_wait<interface_msgs::srv::CameraMovement>(this->modify_camera_client_, request);
}

nlohmann::json CameraCommands::MoveCameraToRobotLocation()
{
    RCLCPP_INFO(node_->get_logger(), "Move camera to robot position.");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    return call_service_empty_response<std_srvs::srv::Trigger>(
        this->move_camera_to_robot_location_client_, request, cmd_timeout_);
}

enum class FollowRobotOption
{
    Off,
    Position,
    PositionAndOrientation
};

FollowRobotOption StringToFollowRobotOption(const std::string &option)
{
    if(option == "off")
    {
        return FollowRobotOption::Off;
    }
    else if(option == "position")
    {
        return FollowRobotOption::Position;
    }
    else if(option == "position_and_orientation")
    {
        return FollowRobotOption::PositionAndOrientation;
    }
    else
    {
        throw InvalidParameterValueException(
            "FollowRobot", "Expected values are: off, position, position_and_orientation");
    }
}

void CameraCommands::SetFollowRobotOption(const nlohmann::json &params)
{
    std::string       option      = params.at("FollowRobot");
    FollowRobotOption FollowRobot = StringToFollowRobotOption(option);
    RCLCPP_INFO(node_->get_logger(), "Set follow robot option: %s.", option.c_str());

    auto request = std::make_shared<interface_msgs::srv::FollowRobot::Request>();
    switch(FollowRobot)
    {
    case FollowRobotOption::Off:
        request->follow_type = interface_msgs::srv::FollowRobot::Request::OFF;
        break;
    case FollowRobotOption::Position:
        request->follow_type = interface_msgs::srv::FollowRobot::Request::POSITION;
        break;
    case FollowRobotOption::PositionAndOrientation:
        request->follow_type = interface_msgs::srv::FollowRobot::Request::POSITION_AND_ORIENTATION;
        break;
    default:
        throw InvalidParameterValueException(
            "FollowRobot", "Expected values are: off, position, position_and_orientation");
    }
    call_service_no_wait<interface_msgs::srv::FollowRobot>(this->set_follow_robot_option_client_,
                                                           request);
}

nlohmann::json CameraCommands::SetRobotLocationToCameraPosition()
{
    RCLCPP_INFO(node_->get_logger(), "Set robot location to camera position.");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    return call_service_empty_response<std_srvs::srv::Trigger>(
        this->set_robot_location_to_camera_position_client_, request, cmd_timeout_);
}
