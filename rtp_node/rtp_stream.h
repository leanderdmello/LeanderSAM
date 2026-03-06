// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include "interface_msgs/srv/set_video_resolution_mode.hpp"

#include "../helpers/shared_memory.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

class RTP_Server : public rclcpp::Node
{
public:
    // Gstreamer
    struct Gobject
    {
        GstBuffer  *buffer;
        GstBus     *bus;
        GstElement *appsrc;
        GstElement *pipeline;
    };


    RTP_Server();
    ~RTP_Server();

private:
    void DeleteGobject(Gobject *object);
    void ConstructPipeline(Gobject *object, int width, int height, int bitrate, int port);
    void ReadData(const std_msgs::msg::Empty &msg);
    void Stream(Gobject *object);
    bool TryReadAndCopy(Gobject *object);
    void SetModeHighRes();
    void SetModeLowRes();
    void SetVideoResolutionMode(const std_msgs::msg::String &msg);

    void SetVideoStreamOption(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response>      response);
    void StreamFrameTimer();

    // ROS
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  rtp_data_ready_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr change_resolution_;
    rclcpp::TimerBase::SharedPtr                           stream_timer_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_video_option_;

    // Create high and low resolution pipelines
    Gobject high_res_mode_;
    Gobject low_res_mode_;

    // shared memory
    int        fd_;
    ShmLayout *frame_;

    // Default high resolution
    uint8_t current_mode_{interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES};
    int     hr_frame_width_;
    int     hr_frame_height_;
    int     hr_bitrate_;

    int lr_frame_width_;
    int lr_frame_height_;
    int lr_bitrate_;

    int fps_;
    int frame_count_{0};

    std::string gstreamer_pipeline_;

    std::mutex mutex_;

    bool paused_{false};
};
