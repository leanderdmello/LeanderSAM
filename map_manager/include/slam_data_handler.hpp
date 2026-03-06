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
#include <vector>

// Third-party libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Project Headers
#include "pointcloud_file_helper.hpp"
#include "pointcloud_shm_helper.hpp"

class SlamDataHandler
{
public:
    SlamDataHandler(const rclcpp::Logger &logger);
    ~SlamDataHandler();
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ConvertRosMessage(
        const sensor_msgs::msg::PointCloud2::SharedPtr message);
    void Backup(const std::filesystem::path &save_map_dir);
    void SharePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);
    void SavePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message,
                        std::filesystem::path                          save_map_dir);
    void SavePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud,
                        std::filesystem::path                            save_map_dir);
    void StartPath(const std::filesystem::path &save_map_dir);
    void SavePath(const nav_msgs::msg::Path::SharedPtr message, std::filesystem::path save_map_dir);
    std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> GetPointCloud(
        const std::filesystem::path &map_dir);
    std::vector<nav_msgs::msg::Path>   GetPathClouds(const std::filesystem::path &trajectory_dir);
    std::optional<nav_msgs::msg::Path> GetPathCloud(const std::filesystem::path &trajectory_dir);

private:
    PointCloudSHMHelper  shm_helper_;
    PointCloudFileHelper file_helper_;
};
