// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "rtp_stream.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <regex>

RTP_Server::RTP_Server() : Node("rtp_streaming_node")
{
    RCLCPP_INFO(this->get_logger(),
                "To watch the stream, run\n gst-launch-1.0 -v udpsrc port=5000 "
                "caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" ! "
                "rtph264depay ! avdec_h264 ! videoconvert ! autovideosink");

    // Declare parameters
    this->declare_parameter("lowres_frame_width", 640);
    this->declare_parameter("lowres_frame_height", 480);
    this->declare_parameter("lowres_bitrate", 1500);
    this->declare_parameter("highres_frame_width", 960);
    this->declare_parameter("highres_frame_height", 720);
    this->declare_parameter("highres_bitrate", 2500);
    this->declare_parameter("frames_per_second", 25);
    this->declare_parameter(
        "pipeline",
        "appsrc name=appsrc is-live=true block=true format=time "
        "caps=video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast intra-refresh=true "
        "sliced-threads=true byte-stream=true vbv-buf-capacity={bitrate_div_fps} bframes=0 "
        "b-adapt=0 ref=1 key-int-max={fps} ! "
        "video/x-h264, profile=baseline ! "
        "rtph264pay pt=96 ! "
        "udpsink host=127.0.0.1 port={port}");

    // Read settings from configuration file
    lr_frame_width_     = this->get_parameter("lowres_frame_width").as_int();
    lr_frame_height_    = this->get_parameter("lowres_frame_height").as_int();
    lr_bitrate_         = this->get_parameter("lowres_bitrate").as_int();
    hr_frame_width_     = this->get_parameter("highres_frame_width").as_int();
    hr_frame_height_    = this->get_parameter("highres_frame_height").as_int();
    hr_bitrate_         = this->get_parameter("highres_bitrate").as_int();
    fps_                = this->get_parameter("frames_per_second").as_int();
    gstreamer_pipeline_ = this->get_parameter("pipeline").as_string();

    gst_init(NULL, NULL);

    ConstructPipeline(&high_res_mode_, hr_frame_width_, hr_frame_height_, hr_bitrate_, 5000);
    ConstructPipeline(&low_res_mode_, lr_frame_width_, lr_frame_height_, lr_bitrate_, 5001);

    // Default launch high res pipeline
    SetModeHighRes();

    // Create timer to stream frames at constant FPS
    auto timer_callback = std::bind(&RTP_Server::StreamFrameTimer, this);
    stream_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / fps_), timer_callback);

    // ROS subscribers
    rtp_data_ready_ = this->create_subscription<std_msgs::msg::Empty>(
        "rtppay", 1, std::bind(&RTP_Server::ReadData, this, std::placeholders::_1));

    // ROS services
    change_resolution_ = this->create_subscription<std_msgs::msg::String>(
        "resolution_changed", 10,
        std::bind(&RTP_Server::SetVideoResolutionMode, this, std::placeholders::_1));

    set_video_option_ = this->create_service<std_srvs::srv::SetBool>(
        "set_video_stream", std::bind(&RTP_Server::SetVideoStreamOption, this,
                                      std::placeholders::_1, std::placeholders::_2));
}

RTP_Server::~RTP_Server()
{
    DeleteGobject(&high_res_mode_);
    DeleteGobject(&low_res_mode_);
    // remove shared memory file descriptor
    shm_close(frame_, fd_);
}

void RTP_Server::ConstructPipeline(Gobject *object, int width, int height, int bitrate, int port)
{
    std::unique_lock<std::mutex> mlock(mutex_);

    // appsrc -> videoconvert -> x264enc -> rtph264pay -> udpsink
    std::string pipeline_desc = gstreamer_pipeline_;
    pipeline_desc =
        std::regex_replace(pipeline_desc, std::regex("\\{width\\}"), std::to_string(width));
    pipeline_desc =
        std::regex_replace(pipeline_desc, std::regex("\\{height\\}"), std::to_string(height));
    pipeline_desc =
        std::regex_replace(pipeline_desc, std::regex("\\{bitrate\\}"), std::to_string(bitrate));
    pipeline_desc = std::regex_replace(pipeline_desc, std::regex("\\{bitrate_div_fps\\}"),
                                       std::to_string(bitrate / fps_));
    pipeline_desc =
        std::regex_replace(pipeline_desc, std::regex("\\{fps\\}"), std::to_string(fps_));
    pipeline_desc =
        std::regex_replace(pipeline_desc, std::regex("\\{port\\}"), std::to_string(port));

    GError *error    = nullptr;
    object->pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);
    if(!object->pipeline)
    {
        RCLCPP_ERROR(this->get_logger(), "Pipeline creation failed: %s", error->message);
        g_error_free(error);
        throw std::runtime_error("Could not create gst pipeline");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Gstreamer pipeline created: %s", pipeline_desc.c_str());
    }

    object->appsrc = gst_bin_get_by_name(GST_BIN(object->pipeline), "appsrc");
    if(!object->appsrc)
        throw std::runtime_error("Could not create gst appsrc");

    // Init buffer on high res mode
    int buffer_size = width * height * 3;
    object->buffer  = gst_buffer_new_allocate(nullptr, buffer_size, nullptr);
    if(!object->buffer)
        throw std::runtime_error("Could not create gst buffer");

    gst_element_set_state(object->pipeline, GST_STATE_READY);

    object->bus = gst_element_get_bus(object->pipeline);
    if(!object->bus)
        throw std::runtime_error("Could not create gst bus");

    RCLCPP_INFO(this->get_logger(), "Streaming to udp://127.0.0.1: %u", port);
}

void RTP_Server::DeleteGobject(Gobject *object)
{
    // Cleanup gstreamer objects
    gst_element_set_state(object->pipeline, GST_STATE_NULL);

    gst_object_unref(object->bus);
    gst_buffer_unref(object->buffer);
    gst_object_unref(object->appsrc);
    gst_object_unref(object->pipeline);
}

void RTP_Server::Stream(Gobject *object)
{
    // Set timestamp
    GST_BUFFER_PTS(object->buffer) = gst_util_uint64_scale(frame_count_++ * GST_SECOND, 1, fps_);
    GST_BUFFER_DURATION(object->buffer) = gst_util_uint64_scale(GST_SECOND, 1, fps_);

    GstFlowReturn ret;
    g_signal_emit_by_name(object->appsrc, "push-buffer", object->buffer, &ret);

    if(ret != GST_FLOW_OK)
        RCLCPP_ERROR(this->get_logger(), "Error streaming buffer!");
}

void RTP_Server::ReadData(const std_msgs::msg::Empty &msg)
{
    (void)msg;

    // nothing to do when paused
    if(paused_)
        return;

    // open shared memory on first received frame
    if(!frame_)
        frame_ = shm_open(DEFAULT_RTP_SHARED_MEMORY, fd_, DEFAULT_RTP_MEMORY_SIZE);

    // Select the right buffer object without holding lock for long
    Gobject *object;
    (current_mode_ == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES) ?
        object = &high_res_mode_ :
        object = &low_res_mode_;

    // TryReadAndCopy handles its own locking for the critical section
    TryReadAndCopy(object);
}

bool RTP_Server::TryReadAndCopy(Gobject *object)
{
    using namespace std::chrono_literals;

    const auto retry = [&]()
    {
        // yield puts this process at the back of the thread queue with same priority.
        std::this_thread::yield();
        // In case there is no other thread in the queue with the same priority we need a sleep.
        std::this_thread::sleep_for(10ms);
    };

    // sequence lock read loop
    for(;;)
    {
        uint64_t before = frame_->seq.load(std::memory_order_acquire);
        if(before & 1ULL)
        {
            // publiser is writing data, retry later.
            retry();
            continue;
        }

        size_t len = frame_->data_len.load(std::memory_order_relaxed);
        if(len > DEFAULT_RTP_MEMORY_SIZE)
        {
            return false;
        }

        // Indicate we are about to copy (so publisher will wait)
        frame_->subscriber_busy.store(1, std::memory_order_release);

        // Check that publisher didn't start between reading seq and setting busy
        uint64_t before2 = frame_->seq.load(std::memory_order_acquire);
        if(before2 != before || (before2 & 1ULL))
        {
            // writer race condition encountered clear busy and retry
            frame_->subscriber_busy.store(0, std::memory_order_release);
            retry();
            continue;
        }

        // Lock only for the critical section: making buffer writable and filling it
        std::unique_lock<std::mutex> mlock(mutex_);

        // Safe to copy the data into gstreamer buffer
        if(!gst_buffer_is_writable(object->buffer))
        {
            GstBuffer *writable = gst_buffer_make_writable(object->buffer);
            if(!writable)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to make buffer writable!");
                return GST_FLOW_ERROR;
            }
            object->buffer = writable;
        }

        gst_buffer_fill(object->buffer, 0, shm_data_ptr(frame_), len);

        mlock.unlock();

        // We're done copying to internal buffer -> clear busy
        frame_->subscriber_busy.store(0, std::memory_order_release);

        // Check if seq didn't change while we copied (reads consistent)
        uint64_t after = frame_->seq.load(std::memory_order_acquire);
        if(before == after)
        {
            // Success:
            return true;
        }
        else
        {
            // Unexpected race condition.
            // Publisher wrote while we copied so try again.
            retry();
            continue;
        }
    }
}

void RTP_Server::SetVideoStreamOption(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    // Video stream option true equals to pause is false
    paused_ = request->data;
    paused_ = !paused_;
    RCLCPP_INFO(this->get_logger(), "Set Stream state to: %d", paused_);

    if(current_mode_ == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES)
    {
        if(paused_)
            gst_element_set_state(high_res_mode_.pipeline, GST_STATE_PAUSED);
        else
            gst_element_set_state(high_res_mode_.pipeline, GST_STATE_PLAYING);
    }
    else if(current_mode_ == interface_msgs::srv::SetVideoResolutionMode::Request::LOWRES)
    {
        if(paused_)
            gst_element_set_state(low_res_mode_.pipeline, GST_STATE_PAUSED);
        else
            gst_element_set_state(low_res_mode_.pipeline, GST_STATE_PLAYING);
    }
    response->success = true;
}

void RTP_Server::SetModeHighRes()
{
    if(!paused_)
    {
        gst_element_set_state(low_res_mode_.pipeline, GST_STATE_PAUSED);
        gst_element_set_state(high_res_mode_.pipeline, GST_STATE_PLAYING);
    }
    frame_count_ = 0;
}

void RTP_Server::SetModeLowRes()
{
    if(!paused_)
    {
        gst_element_set_state(high_res_mode_.pipeline, GST_STATE_PAUSED);
        gst_element_set_state(low_res_mode_.pipeline, GST_STATE_PLAYING);
    }
    frame_count_ = 0;
}

void RTP_Server::SetVideoResolutionMode(const std_msgs::msg::String &msg)
{
    uint8_t mode = static_cast<uint8_t>(std::atoi(msg.data.c_str()));

    if(mode == current_mode_)
        return;

    current_mode_ = mode;

    if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES)
    {
        RCLCPP_INFO(this->get_logger(), "Set video resolution mode: high resolution.");
        SetModeHighRes();
    }
    else if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::LOWRES)
    {
        RCLCPP_INFO(this->get_logger(), "Set video resolution mode: low resolution.");
        SetModeLowRes();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Set video resolution mode: invalid.");
        return;
    }
}

void RTP_Server::StreamFrameTimer()
{
    // nothing to do when paused
    if(paused_)
        return;

    std::unique_lock<std::mutex> mlock(mutex_);

    Gobject *object;
    (current_mode_ == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES) ?
        object = &high_res_mode_ :
        object = &low_res_mode_;

    // Stream the current buffer at constant FPS (newly updated or previous frame)
    Stream(object);
}
