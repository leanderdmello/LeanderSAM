// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "slam_data_handler.hpp"

// C++ Standard Library
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

SlamDataHandler::SlamDataHandler(const rclcpp::Logger &logger)
    : shm_helper_(logger), file_helper_(logger)
{
}

SlamDataHandler::~SlamDataHandler() {}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> SlamDataHandler::ConvertRosMessage(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*message, *pointcloud);
    return pointcloud;
}

void SlamDataHandler::Backup(const std::filesystem::path &save_map_dir)
{
    file_helper_.Backup(save_map_dir);
}

void SlamDataHandler::SharePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud)
{
    shm_helper_.SavePointCloud(pointcloud);
}

void SlamDataHandler::SavePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message,
                                     std::filesystem::path                          save_map_dir)
{
    const auto pointcloud = ConvertRosMessage(message);
    file_helper_.SavePointCloud(pointcloud, save_map_dir);
}

void SlamDataHandler::SavePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pointcloud,
                                     std::filesystem::path                            save_map_dir)
{
    file_helper_.SavePointCloud(pointcloud, save_map_dir);
}

void SlamDataHandler::SavePath(const nav_msgs::msg::Path::SharedPtr message,
                               std::filesystem::path                save_map_dir)
{
    file_helper_.SavePath(message, save_map_dir);
}

void SlamDataHandler::StartPath(const std::filesystem::path &save_map_dir)
{
    file_helper_.StartPath(save_map_dir);
}

std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> SlamDataHandler::GetPointCloud(
    const std::filesystem::path &map_dir)
{
    const auto pointcloud = file_helper_.GetPointCloud(map_dir);
    if(pointcloud.has_value())
    {
        return pointcloud;
    }
    return file_helper_.GetPointCloud(file_helper_.GetBackupDirectory(map_dir));
}

std::vector<nav_msgs::msg::Path> SlamDataHandler::GetPathClouds(
    const std::filesystem::path &trajectory_dir)
{
    return file_helper_.GetPathClouds(trajectory_dir);
}

std::optional<nav_msgs::msg::Path> SlamDataHandler::GetPathCloud(
    const std::filesystem::path &trajectory_dir)
{
    const auto trajectory = file_helper_.GetPathCloud(trajectory_dir / trajectory_name);
    if(trajectory.has_value())
    {
        return trajectory;
    }
    return file_helper_.GetPathCloud(file_helper_.GetBackupDirectory(trajectory_dir) /
                                     trajectory_name);
}
