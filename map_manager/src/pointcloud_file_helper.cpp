// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "pointcloud_file_helper.hpp"

// C++ Standard Library
#include <chrono>
#include <filesystem>
#include <string_view>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS2 Messages
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"

// TF2
#include "tf2/convert.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    float  x;
    float  y;
    float  z;
    float  intensity;
    float  roll;
    float  pitch;
    float  yaw;
    double time;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(
        float,
        pitch,
        pitch)(float, yaw, yaw)(double, time, time))

PointCloudFileHelper::PointCloudFileHelper(const rclcpp::Logger &logger) : logger_(logger) {}

std::optional<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> PointCloudFileHelper::GetPointCloud(
    const std::filesystem::path &map_dir)
{
    std::filesystem::path pointcloud_file = map_dir / map_name;
    try
    {
        auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_file, *pointcloud) == -1)
        {
            std::filesystem::remove(pointcloud_file);
            RCLCPP_ERROR(logger_, "Found pointcloud cannot be loaded %s.", pointcloud_file.c_str());
            return std::nullopt;
        }
        return pointcloud;
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception when loading pointcloud from %s: %s",
                     pointcloud_file.c_str(), e.what());
    }
    return std::nullopt;
}

std::vector<nav_msgs::msg::Path> PointCloudFileHelper::GetPathClouds(
    const std::filesystem::path &trajectory_dir)
{
    std::vector<nav_msgs::msg::Path> paths;
    if(std::filesystem::exists(trajectory_dir))
    {
        for(const auto &entry : std::filesystem::directory_iterator(trajectory_dir))
        {
            const auto file     = entry.path();
            const auto filename = file.filename();
            if(file.extension() == ".pcd" && filename != trajectory_name && filename != map_name)
            {
                auto path_opt = GetPathCloud(file);
                if(path_opt.has_value())
                {
                    paths.push_back(path_opt.value());
                }
            }
        }
    }
    return paths;
}

std::optional<nav_msgs::msg::Path> PointCloudFileHelper::GetPathCloud(
    const std::filesystem::path &trajectory_file)
{
    try
    {
        auto pclpathcloud = std::make_shared<pcl::PointCloud<PointXYZIRPYT>>();

        if(!std::filesystem::exists(trajectory_file))
        {
            return std::nullopt;
        }
        if(pcl::io::loadPCDFile<PointXYZIRPYT>(trajectory_file, *pclpathcloud) == -1)
        {
            std::filesystem::remove(trajectory_file);
            return std::nullopt;
        }

        nav_msgs::msg::Path path;
        path.poses = std::vector<geometry_msgs::msg::PoseStamped>();
        for(const auto &point : *pclpathcloud)
        {
            geometry_msgs::msg::PoseStamped poseStamped;

            geometry_msgs::msg::Point geopoint;
            geopoint.x                = point.x;
            geopoint.y                = point.y;
            geopoint.z                = point.z;
            poseStamped.pose.position = geopoint;

            tf2::Quaternion quaternion_tf2;
            quaternion_tf2.setRPY(point.roll, point.pitch, point.yaw);
            poseStamped.pose.orientation = tf2::toMsg(quaternion_tf2);
            path.poses.push_back(poseStamped);
        }

        return path;
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception when loading path from %s: %s", trajectory_file.c_str(),
                     e.what());
    }
    return std::nullopt;
}

std::filesystem::path PointCloudFileHelper::GetBackupDirectory(
    const std::filesystem::path &save_map_dir)
{
    return save_map_dir / "backup";
}

void PointCloudFileHelper::Backup(const std::filesystem::path &save_map_dir)
{
    const auto backup_dir             = GetBackupDirectory(save_map_dir);
    const auto map_path               = (save_map_dir / map_name).string();
    const auto trajectory_path        = (save_map_dir / trajectory_name).string();
    const auto backup_map_path        = (backup_dir / map_name).string();
    const auto backup_trajectory_path = (backup_dir / trajectory_name).string();
    try
    {
        std::filesystem::create_directories(backup_dir);
        if(std::filesystem::exists(map_path))
        {
            std::filesystem::copy(map_path, backup_map_path,
                                  std::filesystem::copy_options::overwrite_existing);
        }
        if(std::filesystem::exists(trajectory_path))
        {
            std::filesystem::copy(trajectory_path, backup_trajectory_path,
                                  std::filesystem::copy_options::overwrite_existing);
        }
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception when backing up from %s to %s: %s", map_path.c_str(),
                     backup_map_path.c_str(), e.what());
        RCLCPP_ERROR(logger_, "Exception when backing up from %s to %s: %s",
                     trajectory_path.c_str(), backup_trajectory_path.c_str(), e.what());
    }
}

void PointCloudFileHelper::SavePointCloud(
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud,
    const std::filesystem::path                    &save_map_dir)
{
    const auto out_path = (save_map_dir / map_name).string();
    try
    {
        int res = pcl::io::savePCDFileBinary(out_path, *pointcloud);
        if(res == 0)
        {
            RCLCPP_DEBUG(logger_, "Saved map PCD to %s.", out_path.c_str());
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to save map PCD to %s. return=%d", out_path.c_str(), res);
        }
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception when saving map PCD to %s: %s", out_path.c_str(),
                     e.what());
    }
}

void PointCloudFileHelper::StartPath(const std::filesystem::path &save_map_dir)
{
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count();

    const auto file_name   = std::to_string(time).append("_").append(trajectory_name);
    const auto backup_file = (GetBackupDirectory(save_map_dir) / trajectory_name);
    const auto old_name    = (save_map_dir / trajectory_name);
    const auto new_name    = (save_map_dir / file_name);
    try
    {
        if(std::filesystem::exists(old_name))
        {
            RCLCPP_DEBUG(logger_, "Renaming from %s to %s.", old_name.string().c_str(),
                         new_name.string().c_str());

            std::filesystem::rename(old_name, new_name);
        }
        else if(std::filesystem::exists(backup_file))
        {
            RCLCPP_DEBUG(logger_, "Renaming from %s to %s.", backup_file.string().c_str(),
                         new_name.string().c_str());

            std::filesystem::rename(backup_file, new_name);
        }
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception when renaming path from %s to %s, exception %s.",
                     old_name.c_str(), new_name.c_str(), e.what());
    }
}

void PointCloudFileHelper::SavePath(const nav_msgs::msg::Path::SharedPtr message,
                                    const std::filesystem::path         &save_map_dir)
{
    auto pathCloud = std::make_shared<pcl::PointCloud<PointXYZIRPYT>>();

    // Convert poses to points
    for(const auto &pose : message->poses)
    {
        PointXYZIRPYT point;
        point.x = static_cast<float>(pose.pose.position.x);
        point.y = static_cast<float>(pose.pose.position.y);
        point.z = static_cast<float>(pose.pose.position.z);

        // Convert quaternion to Euler angles
        tf2::Quaternion quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                   pose.pose.orientation.z, pose.pose.orientation.w);
        tf2::Matrix3x3  matrix(quaternion);
        double          roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);

        point.roll  = static_cast<float>(roll);
        point.pitch = static_cast<float>(pitch);
        point.yaw   = static_cast<float>(yaw);

        pathCloud->points.push_back(point);
    }
    pathCloud->width    = pathCloud->points.size();
    pathCloud->height   = 1; // unorganized point cloud
    pathCloud->is_dense = false;

    const auto out_path = (save_map_dir / trajectory_name).string();
    try
    {
        int res = pcl::io::savePCDFileBinary(out_path, *pathCloud);
        if(res == 0)
        {
            RCLCPP_DEBUG(logger_, "Saved path PCD to %s.", out_path.c_str());
        }
        else
        {
            RCLCPP_ERROR(logger_, "Failed to save path PCD to %s. return=%d", out_path.c_str(),
                         res);
        }
    }
    catch(const pcl::IOException &e)
    {
        RCLCPP_ERROR(logger_, "PCL IOException when saving path PCD to %s: %s", out_path.c_str(),
                     e.what());
    }
}
