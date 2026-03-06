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
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Third-party libraries

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages and Services
#include "interface_msgs/msg/point_cloud_available.hpp"
#include "interface_msgs/msg/robot_status.hpp"
#include "interface_msgs/msg/trajectories.hpp"
#include "interface_msgs/srv/action_on_index.hpp"
#include "interface_msgs/srv/get_index.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/update_string_at.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "slam_manager_msgs/srv/set_slam_mode.hpp"
#include "slam_manager_msgs/msg/slam_mode.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/empty.hpp"
#include <std_msgs/msg/string.hpp>

// Project Headers
#include "slam_data_handler.hpp"
#include "states/i_slam_state.hpp"


class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode();

private:
    std::vector<std::string> CheckFolderForFloors();

    // Service callbacks
    void ClearMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
    void GetSoftwareVersions(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                             std::shared_ptr<interface_msgs::srv::Listing::Response>      response);
    void AddFloor(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                  std::shared_ptr<interface_msgs::srv::SetString::Response>      response);
    void DeleteFloor(const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
                     std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response);
    void ClearFloor(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
    void ClearPath(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
    void GetFloors(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                   std::shared_ptr<interface_msgs::srv::Listing::Response>      response);
    void GetMappingFloor(const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
                         std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response);
    void GetViewFloor(const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
                      std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response);
    void SelectViewFloor(const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
                         std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response> response);
    void SelectMappingFloor(
        const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response);
    void SetFloorName(const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
                      std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response);

    // Subscription callback
    void ReceivePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message);
    void ReceivePath(const nav_msgs::msg::Path::SharedPtr message);

    void ChangeSLAMMode(uint8_t new_mode);
    void ChangeMode(State new_mode, const std::string &floor_name = "");
    void OnSLAMModeChange(const std::shared_ptr<slam_manager_msgs::msg::SlamMode> message);
    void OnEnterIdle();
    void OnEnterLocalizing();
    void OnEnterMapping();
    void SaveData();

    rclcpp::CallbackGroup::SharedPtr                                slam_callback_group_;
    rclcpp::CallbackGroup::SharedPtr                                control_callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              clear_map_service_;
    rclcpp::Service<interface_msgs::srv::Listing>::SharedPtr        get_software_versions_service_;
    rclcpp::Service<interface_msgs::srv::SetString>::SharedPtr      add_floor_service_;
    rclcpp::Service<interface_msgs::srv::ActionOnIndex>::SharedPtr  delete_floor_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              clear_floor_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              clear_path_service_;
    rclcpp::Service<interface_msgs::srv::Listing>::SharedPtr        get_floors_service_;
    rclcpp::Service<interface_msgs::srv::GetIndex>::SharedPtr       get_mapping_floor_service_;
    rclcpp::Service<interface_msgs::srv::GetIndex>::SharedPtr       get_view_floor_service_;
    rclcpp::Service<interface_msgs::srv::ActionOnIndex>::SharedPtr  select_view_floor_service_;
    rclcpp::Service<interface_msgs::srv::ActionOnIndex>::SharedPtr  select_mapping_floor_service_;
    rclcpp::Service<interface_msgs::srv::UpdateStringAt>::SharedPtr set_floor_name_service_;
    rclcpp::Publisher<interface_msgs::msg::PointCloudAvailable>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<interface_msgs::msg::PointCloudAvailable>::SharedPtr
                                                                      mapping_pointcloud_publisher_;
    rclcpp::Publisher<interface_msgs::msg::Trajectories>::SharedPtr   trajectory_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                clear_map_publisher_;
    rclcpp::Client<slam_manager_msgs::srv::SetSlamMode>::SharedPtr    slam_set_mode_;
    rclcpp::Subscription<slam_manager_msgs::msg::SlamMode>::SharedPtr slam_mode_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    receive_pointcloud_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr              receive_path_;
    rclcpp::TimerBase::SharedPtr                                      savetimer_;

    std::optional<nav_msgs::msg::Path::SharedPtr>           trajectory_;
    std::optional<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_;
    std::filesystem::path                                   map_path_;
    std::filesystem::path                                   version_file_path_;
    std::chrono::duration<int, std::milli>                  map_save_interval_;
    std::chrono::duration<int, std::milli>                  slam_mode_timeout;

    std::string                      current_map_name_{"map_1"};
    std::shared_ptr<SlamDataHandler> slam_data_handler_ =
        std::make_shared<SlamDataHandler>(get_logger());
    std::unique_ptr<ISLAMState> current_state_      = nullptr;
    State                       slam_mode_          = State::Idle;
    bool                        changing_slam_mode_ = false;
};
