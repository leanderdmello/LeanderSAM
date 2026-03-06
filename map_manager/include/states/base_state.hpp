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
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"

// ROS2 Messages and Services
#include "interface_msgs/msg/robot_status.hpp"
#include "interface_msgs/srv/action_on_index.hpp"
#include "interface_msgs/srv/get_index.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/update_string_at.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/trigger.hpp"

// Project Headers
#include "i_slam_state.hpp"
#include "slam_data_handler.hpp"

class BaseState : public ISLAMState
{
public:
    BaseState(const rclcpp::Logger &logger, const std::filesystem::path &version_file_path);
    virtual ~BaseState() = default;

    std::vector<std::string> CheckFolderForFloors();
    void GetSoftwareVersions(std::shared_ptr<interface_msgs::srv::Listing::Response> response);
    void AddFloor(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                  std::shared_ptr<interface_msgs::srv::SetString::Response>      response) override;
    void ClearMap(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
    void ClearFloor(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
    void ClearPath(std::shared_ptr<std_srvs::srv::Trigger::Response> response) override;
    void GetFloors(std::shared_ptr<interface_msgs::srv::Listing::Response> response) override;
    void GetViewFloor(std::shared_ptr<interface_msgs::srv::GetIndex::Response> response) override;
    void SelectViewFloor(
        const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response) override;
    void SelectLocalizationFloor(
        const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response) override;
    void SetFloorName(
        const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
        std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response) override;

    virtual void GetLocalizationFloor(
        const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response) = 0;
    virtual void DeleteFloor(
        const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response);

    virtual void                                 OnEnter(const std::filesystem::path      &base_path,
                                                         const std::string                &floor_name,
                                                         const std::optional<std::string> &view_floor_name) = 0;
    virtual void                                 OnExit()           = 0;
    virtual std::optional<std::filesystem::path> GetSLAMFloorPath() = 0;
    virtual std::string                          GetSLAMFloorName() = 0;
    std::optional<std::string>                   GetViewFloorName() override;
    virtual bool                                 StreamMapLive()                            = 0;
    virtual bool                                 StreamPathLive()                           = 0;
    virtual bool                                 AcceptMapLive()                            = 0;
    virtual std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> OldMergeMap()    = 0;
    virtual bool                                                           AcceptPathLive() = 0;
    virtual void ChangeSLAMFloorName(const std::string &new_name)                           = 0;
    virtual bool IsSLAMFloor(int index)                                                     = 0;
    std::string  GetFloorName(int index);

protected:
    std::filesystem::path      version_file_path_{};
    std::filesystem::path      base_path_{};
    std::optional<std::string> view_floor_{std::nullopt};
    rclcpp::Logger             logger_;
};
