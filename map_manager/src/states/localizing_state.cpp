// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "states/localizing_state.hpp"

// C++ Standard Library
#include <iostream>

// Project Headers
#include "slam_data_handler.hpp"
#include "exceptions.h"

LocalizingState::LocalizingState(const rclcpp::Logger        &logger,
                                 const std::filesystem::path &version_file_path)
    : BaseState(logger, version_file_path), file_helper_(logger)
{
}

void LocalizingState::GetLocalizationFloor(
    const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    const auto floors = CheckFolderForFloors();
    const auto found  = std::find(floors.begin(), floors.end(), localization_floor_);
    if(found == floors.end())
    {
        throw FloorNotSelected("get localization floor", "localization");
    }

    response->index = std::distance(floors.begin(), found);
    RCLCPP_INFO(logger_, "Get localization floor: %d.", response->index);
}

void LocalizingState::OnEnter(const std::filesystem::path      &base_path,
                              const std::string                &floor_name,
                              const std::optional<std::string> &view_floor_name)
{
    base_path_          = base_path;
    localization_floor_ = floor_name;
    view_floor_         = view_floor_name;

    std::filesystem::path floor_path = base_path_ / localization_floor_;
    original_pointcloud_             = file_helper_.GetPointCloud(floor_path);
}

void LocalizingState::OnExit() {}

void LocalizingState::ChangeSLAMFloorName(const std::string &new_name)
{
    localization_floor_ = new_name;
}

std::optional<std::filesystem::path> LocalizingState::GetSLAMFloorPath()
{
    return base_path_ / localization_floor_;
}

std::string LocalizingState::GetSLAMFloorName()
{
    return localization_floor_;
}

bool LocalizingState::StreamMapLive()
{
    return (localization_floor_ == view_floor_);
}

bool LocalizingState::StreamPathLive()
{
    return (localization_floor_ == view_floor_);
}

bool LocalizingState::AcceptMapLive()
{
    return true;
}

std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> LocalizingState::OldMergeMap()
{
    return original_pointcloud_;
}

bool LocalizingState::AcceptPathLive()
{
    return true;
}

bool LocalizingState::IsSLAMFloor(int index)
{
    const auto floors = CheckFolderForFloors();
    if(index < 0 || index >= static_cast<int>(floors.size()))
    {
        return false;
    }
    return (floors[index] == localization_floor_);
}

State LocalizingState::GetState()
{
    return State::Localizing;
}
