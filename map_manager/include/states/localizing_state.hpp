// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

// C++ Standard Library
#include <optional>
#include <string>

// Project Headers
#include "base_state.hpp"

class LocalizingState : public BaseState
{
public:
    LocalizingState(const rclcpp::Logger &logger, const std::filesystem::path &version_file_path);
    ~LocalizingState() = default;

    // Overrides for BaseState pure virtuals
    void GetLocalizationFloor(
        const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response) override;

    void                                 OnEnter(const std::filesystem::path      &base_path,
                                                 const std::string                &floor_name,
                                                 const std::optional<std::string> &view_floor_name) override;
    void                                 OnExit() override;
    std::string                          GetSLAMFloorName() override;
    std::optional<std::filesystem::path> GetSLAMFloorPath() override;
    bool                                 StreamMapLive() override;
    bool                                 StreamPathLive() override;
    bool                                 AcceptMapLive() override;
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> OldMergeMap() override;
    bool                                                           AcceptPathLive() override;
    void  ChangeSLAMFloorName(const std::string &new_name);
    bool  IsSLAMFloor(int index) override;
    State GetState() override;

private:
    std::string                                                    localization_floor_;
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> original_pointcloud_;
    PointCloudFileHelper                                           file_helper_;
};
