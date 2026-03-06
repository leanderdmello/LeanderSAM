// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "states/idle_state.hpp"

// C++ Standard Library
#include <iostream>

// Project Headers
#include "slam_data_handler.hpp"
#include "exceptions.h"

IdleState::IdleState(const rclcpp::Logger &logger, const std::filesystem::path &version_file_path)
    : BaseState(logger, version_file_path)
{
}

void IdleState::GetLocalizationFloor(
    const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    throw FloorNotSelected("get localization floor", "localization");
}

void IdleState::OnEnter(const std::filesystem::path      &base_path,
                        const std::string                &floor_name,
                        const std::optional<std::string> &view_floor_name)
{
    base_path_  = base_path;
    view_floor_ = view_floor_name;
}

void IdleState::OnExit() {}

std::optional<std::filesystem::path> IdleState::GetSLAMFloorPath()
{
    return std::nullopt;
}

std::string IdleState::GetSLAMFloorName()
{
    return "";
}

void IdleState::ChangeSLAMFloorName(const std::string &) {}

bool IdleState::StreamPathLive()
{
    return false;
}

bool IdleState::StreamMapLive()
{
    return false;
}

bool IdleState::IsSLAMFloor(int)
{
    return false;
}

bool IdleState::AcceptMapLive()
{
    return false;
}

std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> IdleState::OldMergeMap()
{
    return std::nullopt;
}

bool IdleState::AcceptPathLive()
{
    return false;
}

State IdleState::GetState()
{
    return State::Idle;
}
