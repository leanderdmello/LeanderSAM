// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "control_command_handler.hpp"

// C++ Standard Library
#include <map>
#include <optional>
#include <thread>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// Project headers
#include "control_command_server.hpp"
#include "exceptions.hpp"

enum class Commands
{
    AddFloor,
    DeleteFloor,
    ClearFloor,
    GetFloors,
    GetMappingFloor,
    GetViewFloor,
    SelectViewFloor,
    SelectMappingFloor,
    SetFloorName,
    AddMarker,
    RemoveMarker,
    SetMarkerLabel,
    GetMarkers,
    ExportMap,
    ClearMap,
    ModifyCamera,
    MoveCameraToRobotLocation,
    SetFollowRobotOption,
    SetRobotLocationToCameraPosition,
    ReportRobotStatus,
    GetSoftwareVersions,
    SetScaleUnit,
    SetVideoResolutionMode,
    SetVideoStreamAddress,
    SetVideoStreamOption,
    ClearPath
};

const std::map<std::string, Commands> command_map = {
    {"AddFloor", Commands::AddFloor},
    {"DeleteFloor", Commands::DeleteFloor},
    {"ClearFloor", Commands::ClearFloor},
    {"GetFloors", Commands::GetFloors},
    {"GetMappingFloor", Commands::GetMappingFloor},
    {"GetViewFloor", Commands::GetViewFloor},
    {"SelectViewFloor", Commands::SelectViewFloor},
    {"SelectMappingFloor", Commands::SelectMappingFloor},
    {"SetFloorName", Commands::SetFloorName},
    {"AddMarker", Commands::AddMarker},
    {"RemoveMarker", Commands::RemoveMarker},
    {"SetMarkerLabel", Commands::SetMarkerLabel},
    {"GetMarkers", Commands::GetMarkers},
    {"ExportMap", Commands::ExportMap},
    {"ClearMap", Commands::ClearMap},
    {"ModifyCamera", Commands::ModifyCamera},
    {"MoveCameraToRobotLocation", Commands::MoveCameraToRobotLocation},
    {"SetFollowRobotOption", Commands::SetFollowRobotOption},
    {"SetRobotLocationToCameraPosition", Commands::SetRobotLocationToCameraPosition},
    {"ReportRobotStatus", Commands::ReportRobotStatus},
    {"GetSoftwareVersions", Commands::GetSoftwareVersions},
    {"SetScaleUnit", Commands::SetScaleUnit},
    {"SetVideoResolutionMode", Commands::SetVideoResolutionMode},
    {"SetVideoStreamAddress", Commands::SetVideoStreamAddress},
    {"SetVideoStreamOption", Commands::SetVideoStreamOption},
    {"ClearPath", Commands::ClearPath}};

ControlCommandHandler::ControlCommandHandler(std::shared_ptr<ControlCommandServer> server,
                                             rclcpp::Node::SharedPtr               node,
                                             Configuration                         config)
    : server_(server),
      node_(node),
      floor_commands_(node, config),
      marker_commands_(node, config),
      map_commands_(node, config),
      camera_commands_(node, config),
      misc_commands_(node, config),
      stream_commands_(node, config)
{
}

nlohmann::json ControlCommandHandler::HandleCommand(const std::string &message)
{
    std::optional<int> id = std::nullopt;
    nlohmann::json     response{};
    response["jsonrpc"] = "2.0";
    try
    {
        nlohmann::json json    = nlohmann::json::parse(message);
        std::string    command = json.at("method");
        if(json.contains("id"))
        {
            id             = json.at("id");
            response["id"] = id.value();
        }
        std::optional<nlohmann::json> params = std::nullopt;
        if(json.contains("params"))
        {
            params = json.at("params");
        }
        auto result = command_map.find(command);
        if(result == command_map.end())
        {
            throw MethodNotFoundException();
        }
        nlohmann::json cmd_response{};
        switch(result->second)
        {
        case Commands::AddFloor:
            cmd_response = floor_commands_.AddFloor(params.value());
            break;
        case Commands::DeleteFloor:
            cmd_response = floor_commands_.DeleteFloor(params.value());
            break;
        case Commands::ClearFloor:
            cmd_response = floor_commands_.ClearFloor();
            break;
        case Commands::GetFloors:
            cmd_response = floor_commands_.GetFloors();
            break;
        case Commands::GetMappingFloor:
            cmd_response = floor_commands_.GetMappingFloor();
            break;
        case Commands::GetViewFloor:
            cmd_response = floor_commands_.GetViewFloor();
            break;
        case Commands::SelectViewFloor:
            cmd_response = floor_commands_.SelectViewFloor(params.value());
            break;
        case Commands::SelectMappingFloor:
            cmd_response = floor_commands_.SelectMappingFloor(params.value());
            break;
        case Commands::SetFloorName:
            cmd_response = floor_commands_.SetFloorName(params.value());
            break;
        case Commands::AddMarker:
            cmd_response = marker_commands_.AddMarker(params.value());
            break;
        case Commands::RemoveMarker:
            cmd_response = marker_commands_.RemoveMarker(params);
            break;
        case Commands::SetMarkerLabel:
            cmd_response = marker_commands_.SetMarkerLabel(params.value());
            break;
        case Commands::GetMarkers:
            cmd_response = marker_commands_.GetMarkers();
            break;
        case Commands::ExportMap:
            cmd_response = map_commands_.ExportMap(params.value());
            break;
        case Commands::ClearMap:
            cmd_response = map_commands_.ClearMap();
            break;
        case Commands::ModifyCamera:
            camera_commands_.ModifyCamera(params);
            break;
        case Commands::MoveCameraToRobotLocation:
            cmd_response = camera_commands_.MoveCameraToRobotLocation();
            break;
        case Commands::SetFollowRobotOption:
            camera_commands_.SetFollowRobotOption(params.value());
            break;
        case Commands::SetRobotLocationToCameraPosition:
            cmd_response = camera_commands_.SetRobotLocationToCameraPosition();
            break;
        case Commands::ReportRobotStatus:
            misc_commands_.ReportRobotStatus(params.value());
            break;
        case Commands::GetSoftwareVersions:
            cmd_response = misc_commands_.GetSoftwareVersions();
            break;
        case Commands::SetScaleUnit:
            stream_commands_.SetScaleUnit(params.value());
            break;
        case Commands::SetVideoResolutionMode:
            cmd_response = stream_commands_.SetVideoResolutionMode(params.value());
            break;
        case Commands::SetVideoStreamAddress:
            cmd_response = stream_commands_.SetVideoStreamAddress(params.value());
            break;
        case Commands::SetVideoStreamOption:
            cmd_response = stream_commands_.SetVideoStreamOption(params.value());
            break;
        case Commands::ClearPath:
            cmd_response = stream_commands_.ClearPath();
            break;
        default:
            throw NotImplementedException();
        }
        if(!id.has_value())
        {
            response = nullptr;
        }
        else
        {
            response["result"] = cmd_response;
        }
    }
    catch(const nlohmann::json::exception &exception)
    {
        nlohmann::json error_response{};
        error_response["code"]    = 1;
        error_response["message"] = std::string{exception.what()};
        response["error"]         = error_response;
    }
    catch(const JSONRPCException &exception)
    {
        nlohmann::json error_response{};
        error_response["code"]    = exception.code();
        error_response["message"] = std::string{exception.what()};
        response["error"]         = error_response;
    }
    catch(const std::bad_optional_access &exception)
    {
        nlohmann::json error_response{};
        error_response["code"]    = -32602;
        error_response["message"] = "Missing required parameters";
        response["error"]         = error_response;
    }
    return response;
}

void ControlCommandHandler::Run()
{
    while(server_->Running())
    {
        // Get next command waits till a command is available
        auto commands = server_->GetCommands();
        for(const auto &command : commands)
        {
            RCLCPP_INFO(node_->get_logger(), "Received: %s", command.Message.c_str());
            nlohmann::json response = HandleCommand(command.Message);
            if(!response.empty())
            {
                RCLCPP_INFO(node_->get_logger(), "Sending response: %s", response.dump().c_str());
                server_->SendResponse({command.Handle, response.dump()});
            }
        }
    }
}
