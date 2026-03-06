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
#include <string_view>

// Third-party libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Project Headers
#include "../../helpers/shared_memory.h"

constexpr std::string_view trajectory_name = "trajectory.pcd";
constexpr std::string_view map_name        = "map.pcd";

class PointCloudFileHelper
{
public:
    PointCloudFileHelper(const rclcpp::Logger &logger);
    ~PointCloudFileHelper() = default;
    void Backup(const std::filesystem::path &save_map_dir);
    void SavePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud,
                        const std::filesystem::path                    &save_map_dir);
    void SavePath(const nav_msgs::msg::Path::SharedPtr message,
                  const std::filesystem::path         &save_map_dir);
    void StartPath(const std::filesystem::path &save_map_dir);
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> GetPointCloud(
        const std::filesystem::path &map_dir);
    std::vector<nav_msgs::msg::Path>   GetPathClouds(const std::filesystem::path &trajectory_dir);
    std::optional<nav_msgs::msg::Path> GetPathCloud(const std::filesystem::path &trajectory_file);
    std::filesystem::path GetBackupDirectory(const std::filesystem::path &save_map_dir);

private:
private:
    rclcpp::Logger logger_;
    int            file_descriptor_{};
    ShmLayout     *shm_layout_;
};
