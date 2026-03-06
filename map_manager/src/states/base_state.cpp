// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "states/base_state.hpp"
#include "exceptions.h"

// C++ Standard Library
#include <fstream>
#include <iostream>
#include <sstream>

BaseState::BaseState(const rclcpp::Logger &logger, const std::filesystem::path &version_file_path)
    : logger_(logger), version_file_path_(version_file_path)
{
}

std::vector<std::string> BaseState::CheckFolderForFloors()
{
    std::vector<std::string> floors;
    if(std::filesystem::exists(base_path_))
    {
        for(const auto &file : std::filesystem::directory_iterator(base_path_))
        {
            if(std::filesystem::is_directory(file))
            {
                floors.push_back(file.path().filename().string());
            }
        }
    }
    return floors;
}

std::string BaseState::GetFloorName(int index)
{
    const auto floors = CheckFolderForFloors();
    if(index < 0 || index >= static_cast<int>(floors.size()))
    {
        throw IndexOutOfRange("get floor name", index);
    }
    return floors[index];
}

void BaseState::GetSoftwareVersions(
    std::shared_ptr<interface_msgs::srv::Listing::Response> response)
{
    std::ifstream            version_stream(version_file_path_.string());
    std::vector<std::string> versions;
    std::string              line;
    if(version_stream.is_open())
    {
        while(std::getline(version_stream, line))
        {
            if(!line.empty())
            {
                versions.push_back(line);
            }
        }
    }
    version_stream.close();
    response->data = versions;
    std::stringstream stream;
    stream << "Get software versions(";
    for(const auto &version : versions)
    {
        stream << version;
        if(response->data.back() != version)
        {
            stream << ", ";
        }
    }
    stream << ")";
    RCLCPP_INFO(logger_, stream.str().c_str());
}

void BaseState::AddFloor(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                         std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    if(std::find(floors.begin(), floors.end(), request->data) != floors.end())
    {
        throw FileAlreadyExistsException("Floor");
    }

    std::filesystem::path floor_path = base_path_ / request->data;
    std::filesystem::create_directories(floor_path);

    RCLCPP_INFO(logger_, "Add floor %s.", request->data.c_str());
}

void BaseState::ClearMap(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if(std::filesystem::exists(base_path_))
    {
        for(const auto &file : std::filesystem::directory_iterator(base_path_))
        {
            std::filesystem::remove_all(file);
        }
    }
    view_floor_ = std::nullopt;
    RCLCPP_INFO(logger_, "Clear map.");
}

void BaseState::ClearFloor(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if(!view_floor_.has_value())
    {
        throw FloorNotSelected("clear floor", "view");
    }

    std::filesystem::path floor_path = base_path_ / view_floor_.value();
    if(std::filesystem::exists(floor_path))
    {
        for(const auto &file : std::filesystem::directory_iterator(floor_path))
        {
            std::filesystem::remove_all(file);
        }
    }
    RCLCPP_INFO(logger_, "Clear floor %s.", view_floor_.value().c_str());
}

void BaseState::ClearPath(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if(!view_floor_.has_value())
    {
        throw FloorNotSelected("clear floor", "view");
    }

    std::filesystem::path floor_path = base_path_ / view_floor_.value();
    for(const auto &entry : std::filesystem::directory_iterator(floor_path))
    {
        const auto file     = entry.path();
        const auto filename = file.filename();
        if(file.extension() == ".pcd" && filename != map_name)
        {
            std::filesystem::remove_all(file);
        }
    }
    RCLCPP_INFO(logger_, "Clear path.");
}

void BaseState::GetFloors(std::shared_ptr<interface_msgs::srv::Listing::Response> response)
{
    response->data = CheckFolderForFloors();
    std::stringstream stream;
    stream << "Get floors(";
    for(const auto &floor : response->data)
    {
        stream << floor;
        if(response->data.back() != floor)
        {
            stream << ", ";
        }
    }
    stream << ")";
    RCLCPP_INFO(logger_, stream.str().c_str());
}

void BaseState::GetViewFloor(std::shared_ptr<interface_msgs::srv::GetIndex::Response> response)
{
    if(view_floor_.has_value())
    {
        const auto floors = CheckFolderForFloors();
        const auto found  = std::find(floors.begin(), floors.end(), view_floor_.value());
        if(found == floors.end())
        {
            throw FloorNotSelected("get view floor", "view");
        }
        response->index = std::distance(floors.begin(), found);
        RCLCPP_INFO(logger_, "Get view floor %d.", response->index);
    }
    else
    {
        throw FloorNotSelected("get view floor", "view");
    }
}

void BaseState::SelectViewFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    if(request->index >= static_cast<int>(floors.size()) || request->index < 0)
    {
        throw IndexOutOfRange("select view floor", request->index);
    }
    view_floor_ = floors[request->index];
    RCLCPP_INFO(logger_, "select view floor %d.", request->index);
}

void BaseState::SelectLocalizationFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    if(request->index >= static_cast<int>(floors.size()) || request->index < 0)
    {
        throw IndexOutOfRange("select localization floor", request->index);
    }

    RCLCPP_INFO(logger_, "select localization floor %d.", request->index);
}

void BaseState::SetFloorName(
    const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
    std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    if(request->index >= static_cast<int>(floors.size()) || request->index < 0)
    {
        throw IndexOutOfRange("set floor name", request->data, request->index);
    }
    if(std::filesystem::exists(base_path_ / request->data))
    {
        throw FileAlreadyExistsException(request->data);
    }
    std::string floor_name = floors[request->index];
    std::filesystem::rename(base_path_ / floor_name, base_path_ / request->data);
    if(floor_name == view_floor_)
    {
        view_floor_ = request->data;
    }
    RCLCPP_INFO(logger_, "Set floor name %s at index %d.", request->data.c_str(), request->index);
}

void BaseState::DeleteFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    int        index  = request->index;
    const auto floors = CheckFolderForFloors();
    if(floors.size() <= static_cast<size_t>(index))
    {
        throw IndexOutOfRange("delete floor", request->index);
    }
    std::string           floor_name = floors[index];
    std::filesystem::path floor_path = base_path_ / floor_name;
    std::filesystem::remove_all(base_path_ / floor_name);

    RCLCPP_INFO(logger_, "Delete floor %d.", index);
}

std::optional<std::string> BaseState::GetViewFloorName()
{
    return view_floor_;
}
