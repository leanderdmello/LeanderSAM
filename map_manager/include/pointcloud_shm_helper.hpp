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

class PointCloudSHMHelper
{
public:
    PointCloudSHMHelper(const rclcpp::Logger &logger);
    ~PointCloudSHMHelper();
    void WaitUntilSubscriberNotBusy();
    void SavePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud);

private:
    rclcpp::Logger logger_;
    int            file_descriptor_{};
    ShmLayout     *shm_layout_;
};
