// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "states/mapping_state.hpp"

// C++ Standard Library
#include <iostream>

// Project Headers
#include "slam_data_handler.hpp"
#include "exceptions.h"

MappingState::MappingState(const rclcpp::Logger        &logger,
                           const std::filesystem::path &version_file_path)
    : BaseState(logger, version_file_path)
{
}

void MappingState::GetLocalizationFloor(
    const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    const auto found  = std::find(floors.begin(), floors.end(), mapping_floor_);
    if(found == floors.end())
    {
        throw FloorNotSelected("get mapping floor", "mapping");
    }

    response->index = std::distance(floors.begin(), found);
    RCLCPP_INFO(logger_, "Get mapping floor: %d.", response->index);
}

void MappingState::OnEnter(const std::filesystem::path      &base_path,
                           const std::string                &floor_name,
                           const std::optional<std::string> &view_floor_name)
{
    base_path_     = base_path;
    mapping_floor_ = floor_name;
    view_floor_    = view_floor_name;
}

void MappingState::OnExit() {}

void MappingState::ChangeSLAMFloorName(const std::string &new_name)
{
    mapping_floor_ = new_name;
}

std::optional<std::filesystem::path> MappingState::GetSLAMFloorPath()
{
    return base_path_ / mapping_floor_;
}

std::string MappingState::GetSLAMFloorName()
{
    return mapping_floor_;
}

bool MappingState::StreamMapLive()
{
    return (mapping_floor_ == view_floor_);
}

bool MappingState::StreamPathLive()
{
    return (mapping_floor_ == view_floor_);
}

bool MappingState::AcceptMapLive()
{
    return true;
}

std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> MappingState::OldMergeMap()
{
    return std::nullopt;
}

bool MappingState::AcceptPathLive()
{
    return true;
}

bool MappingState::IsSLAMFloor(int index)
{
    const auto floors = CheckFolderForFloors();
    if(index < 0 || index >= static_cast<int>(floors.size()))
    {
        return false;
    }
    return (floors[index] == mapping_floor_);
}

State MappingState::GetState()
{
    return State::Mapping;
}
