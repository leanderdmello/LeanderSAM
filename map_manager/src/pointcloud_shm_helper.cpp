// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "pointcloud_shm_helper.hpp"

// C++ Standard Library
#include <chrono>
#include <thread>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

PointCloudSHMHelper::PointCloudSHMHelper(const rclcpp::Logger &logger) : logger_(logger)
{
    shm_create(DEFAULT_PCD_SHARED_MEMORY, file_descriptor_, DEFAULT_PCD_MEMORY_SIZE);
    shm_layout_ = shm_open(DEFAULT_PCD_SHARED_MEMORY, file_descriptor_, DEFAULT_PCD_MEMORY_SIZE);
}

PointCloudSHMHelper::~PointCloudSHMHelper()
{
    shm_close(shm_layout_, file_descriptor_);
    shm_destroy(DEFAULT_PCD_SHARED_MEMORY);
}

void PointCloudSHMHelper::WaitUntilSubscriberNotBusy()
{
    using namespace std::chrono_literals;
    while(shm_layout_->subscriber_busy.load(std::memory_order_acquire) != 0)
        std::this_thread::sleep_for(100us); // small sleep
}

void PointCloudSHMHelper::SavePointCloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud)
{
    size_t data_len = pointcloud->size() * sizeof(pcl::PointXYZ);
    if(data_len > shm_layout_->data_capacity)
    {
        size_t new_capacity = data_len * 2;
        shm_close(shm_layout_, file_descriptor_);
        shm_create(DEFAULT_PCD_SHARED_MEMORY, file_descriptor_, new_capacity);
        shm_layout_ = shm_open(DEFAULT_PCD_SHARED_MEMORY, file_descriptor_, new_capacity);

        data_len = new_capacity;
    }

    // Note: seqlock is incremented for even/odd checks. Could be changed to fetch-add
    // fetch-sub, but uint64_t is big enough to never cause issues.

    // Wait until subscriber isn't copying to its internal buffer
    WaitUntilSubscriberNotBusy();

    // Increment seq to mark writer-in-progress (make it odd).
    shm_layout_->seq.fetch_add(1, std::memory_order_acq_rel);

    // Write payload
    shm_layout_->data_len.store(data_len, std::memory_order_relaxed);
    std::memcpy(shm_data_ptr(shm_layout_), pointcloud->data(), data_len);

    // Ensure writes visible before completing write
    std::atomic_thread_fence(std::memory_order_release);

    // Increment seq (make it even, indicating stable)
    shm_layout_->seq.fetch_add(1, std::memory_order_acq_rel);
}
