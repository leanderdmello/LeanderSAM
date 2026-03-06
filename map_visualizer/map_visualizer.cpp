// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "map_visualizer.h"

#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <cstring>
#include <iostream>
#include <math.h>
#include <set>
#include <vector>

Map_Visualizer::Map_Visualizer() : Node("simple_Map_Visualizer")
{
    // Declare parameters
    this->declare_parameter("lowres_frame_width", 640);
    this->declare_parameter("lowres_frame_height", 480);
    this->declare_parameter("highres_frame_width", 960);
    this->declare_parameter("highres_frame_height", 720);
    this->declare_parameter("sd_card_storage_path", "/home/avular/external_storage/sd-card");

    // Read settings from configuration fileshm
    lr_frame_width_  = this->get_parameter("lowres_frame_width").as_int();
    lr_frame_height_ = this->get_parameter("lowres_frame_height").as_int();
    hr_frame_width_  = this->get_parameter("highres_frame_width").as_int();
    hr_frame_height_ = this->get_parameter("highres_frame_height").as_int();
    sd_card_path_    = this->get_parameter("sd_card_storage_path").as_string();

    // Create storage if not present
    sd_card_dir_ = sd_card_path_;
    std::error_code ec;
    if(!std::filesystem::exists(sd_card_dir_))
    {
        if(!std::filesystem::create_directories(sd_card_dir_, ec))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s",
                         ec.message().c_str());
            sd_card_dir_ = "";
        }
    }

    // Renderer will be created on first render timer callback to ensure GL context on correct
    // thread
    renderer_ = nullptr;

    // Gstreamer frame size
    frame_size_ = hr_frame_width_ * hr_frame_height_ * 3;

    // Current path is now simply working dir
    view_path_ = std::filesystem::current_path();

    // Create shared memory object
    shm_create(DEFAULT_PCD_SHARED_MEMORY, map_fd_, DEFAULT_PCD_MEMORY_SIZE);
    map_frame_ = shm_open(DEFAULT_PCD_SHARED_MEMORY, map_fd_, DEFAULT_PCD_MEMORY_SIZE);
    // Create shared memory objects
    shm_create(DEFAULT_RTP_SHARED_MEMORY, rtp_fd_, DEFAULT_RTP_MEMORY_SIZE);
    rtp_frame_ = shm_open(DEFAULT_RTP_SHARED_MEMORY, rtp_fd_, DEFAULT_RTP_MEMORY_SIZE);

    // Ros publisher
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("rtppay", 1);

    // Create separate callback groups for each subscription to allow concurrent handling
    // Each group is mutually exclusive to ensure sequential processing of messages on the same
    // topic
    pc_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    pc_localization_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tr_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    clear_map_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    robot_odometry_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscription options with callback groups
    auto pc_sub_options           = rclcpp::SubscriptionOptions();
    pc_sub_options.callback_group = pc_callback_group_;

    auto pc_loc_sub_options           = rclcpp::SubscriptionOptions();
    pc_loc_sub_options.callback_group = pc_localization_callback_group_;

    auto tr_sub_options           = rclcpp::SubscriptionOptions();
    tr_sub_options.callback_group = tr_callback_group_;

    auto clear_map_sub_options           = rclcpp::SubscriptionOptions();
    clear_map_sub_options.callback_group = clear_map_callback_group_;

    auto robot_odom_sub_options           = rclcpp::SubscriptionOptions();
    robot_odom_sub_options.callback_group = robot_odometry_callback_group_;

    pc_subscription_ = this->create_subscription<interface_msgs::msg::PointCloudAvailable>(
        "point_cloud_available", 1,
        std::bind(&Map_Visualizer::ReadPointCloudData, this, std::placeholders::_1),
        pc_sub_options);
    pc_localization_subscription_ =
        this->create_subscription<interface_msgs::msg::PointCloudAvailable>(
            "mapping_point_cloud_available", 1,
            std::bind(&Map_Visualizer::ProcessLocalizationPath, this, std::placeholders::_1),
            pc_loc_sub_options);
    tr_subscription_ = this->create_subscription<interface_msgs::msg::Trajectories>(
        "trajectories", 1,
        std::bind(&Map_Visualizer::ReadTrajectoryData, this, std::placeholders::_1),
        tr_sub_options);

    clear_map_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "clear_map", 1, std::bind(&Map_Visualizer::ClearMap, this, std::placeholders::_1),
        clear_map_sub_options);

    signal_resolution_ = this->create_publisher<std_msgs::msg::String>("resolution_changed", 1);

    resolution_ = this->create_service<interface_msgs::srv::SetVideoResolutionMode>(
        "set_video_resolution_mode", std::bind(&Map_Visualizer::SetResolutionMode, this,
                                               std::placeholders::_1, std::placeholders::_2));

    robot_initial_pose_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

    robot_odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/slam/mapping/odometry", 1,
        std::bind(&Map_Visualizer::UpdateRobotPose, this, std::placeholders::_1),
        robot_odom_sub_options);

    using namespace std::chrono_literals;
    last_odometry_time_ = this->now();

    // Odometry watchdog timer - runs on ROS executor threads
    odometry_watchdog_timer_ = this->create_wall_timer(
        500ms,
        [this]()
        {
            auto now = this->now();
            if((now - last_odometry_time_).seconds() > 1.0)
            {
                EnqueueRenderCommand([this]() { renderer_->ResetRobotPosition(); });
            }
        });

    // ROS services
    camera_movement_ = this->create_service<interface_msgs::srv::CameraMovement>(
        "modify_camera", std::bind(&Map_Visualizer::UpdateCameraPosition, this,
                                   std::placeholders::_1, std::placeholders::_2));

    move_camera_to_robot_ = this->create_service<std_srvs::srv::Trigger>(
        "move_camera_to_robot", std::bind(&Map_Visualizer::MoveCameraToRobot, this,
                                          std::placeholders::_1, std::placeholders::_2));

    follow_robot_ = this->create_service<interface_msgs::srv::FollowRobot>(
        "follow_robot", std::bind(&Map_Visualizer::FollowRobot, this, std::placeholders::_1,
                                  std::placeholders::_2));

    set_robot_position_to_camera_position_ = this->create_service<std_srvs::srv::Trigger>(
        "set_robot_location_to_camera",
        std::bind(&Map_Visualizer::SetRobotPositionToCameraPosition, this, std::placeholders::_1,
                  std::placeholders::_2));

    scale_unit_meters_ = this->create_service<std_srvs::srv::SetBool>(
        "scale_unit_meters", std::bind(&Map_Visualizer::SetScaleUnitMeters, this,
                                       std::placeholders::_1, std::placeholders::_2));

    add_marker_ = this->create_service<interface_msgs::srv::SetString>(
        "add_marker",
        std::bind(&Map_Visualizer::AddMarker, this, std::placeholders::_1, std::placeholders::_2));

    remove_marker_ = this->create_service<interface_msgs::srv::ActionOnOptionalIndex>(
        "remove_marker", std::bind(&Map_Visualizer::RemoveMarker, this, std::placeholders::_1,
                                   std::placeholders::_2));

    get_markers_ = this->create_service<interface_msgs::srv::Listing>(
        "get_markers",
        std::bind(&Map_Visualizer::GetMarkers, this, std::placeholders::_1, std::placeholders::_2));

    set_marker_label_ = this->create_service<interface_msgs::srv::SetString>(
        "set_marker_label", std::bind(&Map_Visualizer::SetMarkerLabel, this, std::placeholders::_1,
                                      std::placeholders::_2));

    set_marker_label_at_ = this->create_service<interface_msgs::srv::UpdateStringAt>(
        "set_marker_label_at", std::bind(&Map_Visualizer::SetMarkerLabelAt, this,
                                         std::placeholders::_1, std::placeholders::_2));

    export_map_ = this->create_service<interface_msgs::srv::SetString>(
        "export_map",
        std::bind(&Map_Visualizer::ExportMap, this, std::placeholders::_1, std::placeholders::_2));
}

Map_Visualizer::~Map_Visualizer()
{
    renderer_ = nullptr;

    if(rtp_frame_)
    {
        shm_close(rtp_frame_, rtp_fd_);
        shm_destroy(DEFAULT_RTP_SHARED_MEMORY);
    }

    if(map_frame_)
        shm_close(map_frame_, map_fd_);
}

void Map_Visualizer::CleanupGLResources()
{
    renderer_ = nullptr;
}

void Map_Visualizer::RequestRender()
{
    render_requested_.store(true, std::memory_order_release);
    render_cv_.notify_one();
}

void Map_Visualizer::EnqueueRenderCommand(std::function<void()> command)
{
    {
        std::lock_guard<std::mutex> lock(renderer_mutex_);
        render_commands_.push(std::move(command));
    }
    RequestRender();
}

template <typename R>
R Map_Visualizer::ExecuteOnRenderThread(std::function<R()> command)
{
    std::promise<R> promise;
    std::future<R>  future = promise.get_future();

    EnqueueRenderCommand(
        [&promise, command]()
        {
            try
            {
                promise.set_value(command());
            }
            catch(...)
            {
                promise.set_exception(std::current_exception());
            }
        });

    return future.get();
}

// Explicit template instantiations for types used
template bool Map_Visualizer::ExecuteOnRenderThread<bool>(std::function<bool()>);
template int  Map_Visualizer::ExecuteOnRenderThread<int>(std::function<int()>);
template std::tuple<float, float> Map_Visualizer::ExecuteOnRenderThread<std::tuple<float, float>>(
    std::function<std::tuple<float, float>()>);
template std::tuple<glm::vec2, glm::vec4> Map_Visualizer::ExecuteOnRenderThread<
    std::tuple<glm::vec2, glm::vec4>>(std::function<std::tuple<glm::vec2, glm::vec4>()>);
template std::vector<std::string> Map_Visualizer::ExecuteOnRenderThread<std::vector<std::string>>(
    std::function<std::vector<std::string>()>);

#pragma region Push Data
void Map_Visualizer::ReadPointCloudData(const interface_msgs::msg::PointCloudAvailable &msg)
{
    ZoneScoped;
    TracyFiberEnter("ReadPointCloudData");

    auto old_view_path = view_path_;
    view_path_         = std::filesystem::path(msg.floor_path.c_str());
    // Reload markers if the view path has changed
    if(old_view_path != view_path_)
    {
        // Clear existing marker data
        EnqueueRenderCommand(
            [this]()
            {
                renderer_->RemoveAllMarkers();
                marker_map_.clear();
                marker_user_idx_.clear();
                active_marker_idx_ = 0;

                // Only load markers when a valid view path is set
                if(!view_path_.empty())
                {
                    LoadMarkersFromDisk();
                }
            });
    }

    // Capture floor name and view path for the command
    std::string floor_name = view_path_.empty() ? "" : view_path_.filename().string();

    // open shared memory on first received frame
    if(!map_frame_)
        map_frame_ = shm_open(DEFAULT_PCD_SHARED_MEMORY, map_fd_, DEFAULT_PCD_MEMORY_SIZE);

    // Read and copy map data on callback thread (don't block render thread)
    std::vector<float> data;
    if(TryReadAndCopyMap(data))
    {
        RCLCPP_INFO(this->get_logger(), "Map rx: %zu floats for floor '%s'", data.size(),
                    floor_name.c_str());

        // Create or get the map point cloud object if needed
        if(!map_)
            map_ = std::make_unique<PointCloud>(this->get_logger(), hr_frame_width_,
                                                hr_frame_height_, false);

        // Preprocess the map data on THIS thread (CPU-intensive work OFF render thread)
        map_->PreprocessMap(data.data(), data.size() * sizeof(float));

        // Only GPU upload happens on render thread (quick operation)
        EnqueueRenderCommand(
            [this, floor_name]()
            {
                ZoneScoped;
                TracyFiberEnter("ReadPointCloudData");
                if(renderer_ && !floor_name.empty())
                {
                    renderer_->SetFloorName(floor_name);
                }
                // Only GPU upload - preprocessing already done on callback thread
                if(map_)
                {
                    RCLCPP_INFO(this->get_logger(), "Map upload -> GPU (floor '%s')",
                                floor_name.c_str());
                    map_->UploadToGPU(PointCloud::Mode::Map);
                    if(trajectory_)
                    {
                        trajectory_->UseSharedTextures(map_->GetColorTexture());
                    }
                    renderer_->SetPointCloudSources(map_.get(), trajectory_.get());
                }
                TracyFiberLeave;
            });
    }
    else
    {
        // Only update floor name and markers if no new map data
        EnqueueRenderCommand(
            [this, floor_name]()
            {
                ZoneScoped;
                TracyFiberEnter("ReadPointCloudData");
                if(renderer_ && !floor_name.empty())
                {
                    renderer_->SetFloorName(floor_name);
                }
                TracyFiberLeave;
            });
    }

    TracyFiberLeave;
}

void Map_Visualizer::ProcessLocalizationPath(const interface_msgs::msg::PointCloudAvailable &msg)
{
    ZoneScoped;
    TracyFiberEnter("ProcessLocalizationPath");

    localization_floor_path_ = std::filesystem::path(msg.floor_path.c_str());

    TracyFiberLeave;
}

void Map_Visualizer::ReadTrajectoryData(const interface_msgs::msg::Trajectories &msg)
{
    ZoneScoped;
    TracyFiberEnter("ReadTrajectoryData");

    if(msg.trajectories.empty())
    {
        EnqueueRenderCommand([this]() { renderer_->ClearTrajectory(); });
        TracyFiberLeave;
        return;
    }

    // trajectory 0 = is the whole route,
    // trajectory 1 = the rosbag loop we want
    std::string color = "#00ff00";

    // 4 values per point (xyzreflect)
    std::vector<float> data(msg.trajectories.back().poses.size() * 4);

    // cant 1-1 copy
    for(size_t i = 0; i < msg.trajectories.back().poses.size(); i++)
    {
        data[i * 4 + 0] = msg.trajectories.back().poses[i].pose.position.x;
        data[i * 4 + 1] = msg.trajectories.back().poses[i].pose.position.y;
        data[i * 4 + 2] = msg.trajectories.back().poses[i].pose.position.z;
        data[i * 4 + 3] = 0.f; // unused
    }

    // Create or get the trajectory point cloud object if needed
    if(!trajectory_)
        trajectory_ = std::make_unique<PointCloud>(this->get_logger(), hr_frame_width_,
                                                   hr_frame_height_, false);

    // Convert color string (extract RGB values)
    // For now, just preprocess with a default color
    glm::vec3 rgb_color(0.0f, 1.0f, 0.0f); // green for trajectory

    // Preprocess the trajectory data on THIS thread (CPU-intensive work OFF render thread)
    trajectory_->PreprocessTrajectory(rgb_color, data.data(), data.size() * sizeof(float));

    // Only GPU upload happens on render thread (quick operation)
    EnqueueRenderCommand(
        [this, color]()
        {
            // Only GPU upload - preprocessing already done on callback thread
            if(trajectory_)
            {
                RCLCPP_INFO(this->get_logger(), "Trajectory upload -> GPU (%zu points)",
                            trajectory_->GetPointCount());
                if(map_)
                {
                    trajectory_->UseSharedTextures(map_->GetColorTexture());
                }
                trajectory_->UploadToGPU(PointCloud::Mode::Trajectory);
                renderer_->SetPointCloudSources(map_.get(), trajectory_.get());
            }
        });

    TracyFiberLeave;
}

void Map_Visualizer::RenderMap()
{
    ZoneScoped;
    ZoneName("RenderMap", 9);

    // TODO() try not to copy each frame
    auto frame = renderer_->Render();
    WriteData(frame.data(), frame.size());
}

bool Map_Visualizer::TryReadAndCopyMap(std::vector<float> &data)
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
        uint64_t before = map_frame_->seq.load(std::memory_order_acquire);
        if(before & 1ULL)
        {
            // publiser is writing data, retry later.
            retry();
            continue;
        }

        size_t len = map_frame_->data_len.load(std::memory_order_relaxed);
        if(len > DEFAULT_PCD_MEMORY_SIZE)
            return false;

        // Indicate we are about to copy (so publisher will wait)
        map_frame_->subscriber_busy.store(1, std::memory_order_release);

        // Check that publisher didn't start between reading seq and setting busy
        uint64_t before2 = map_frame_->seq.load(std::memory_order_acquire);
        if(before2 != before || (before2 & 1ULL))
        {
            // writer race condition encountered clear busy and retry
            map_frame_->subscriber_busy.store(0, std::memory_order_release);
            retry();
            continue;
        }

        // safe to copy frame
        data.resize(len);
        std::memcpy(data.data(), shm_data_ptr(map_frame_), len);

        // We're done copying to internal buffer -> clear busy
        map_frame_->subscriber_busy.store(0, std::memory_order_release);

        // Check if seq didn't change while we copied (reads consistent)
        uint64_t after = map_frame_->seq.load(std::memory_order_acquire);
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

void Map_Visualizer::WaitUntilSubscriberNotBusy()
{
    using namespace std::chrono_literals;
    while(rtp_frame_->subscriber_busy.load(std::memory_order_acquire) != 0)
        std::this_thread::sleep_for(100us); // small sleep
}

void Map_Visualizer::RenderLoop()
{
    ZoneScoped;
    tracy::SetThreadName("RenderThread");

    using namespace std::chrono_literals;

    RCLCPP_INFO(this->get_logger(), "Render thread started");

    // Create renderer on this thread to bind OpenGL context
    {
        std::lock_guard<std::mutex> lock(renderer_mutex_);
        if(!renderer_)
        {
            RCLCPP_INFO(this->get_logger(), "Creating renderer on render thread...");
            renderer_ = std::make_unique<Map_Renderer>(this->get_logger(), hr_frame_width_,
                                                       hr_frame_height_);
            RCLCPP_INFO(this->get_logger(), "Renderer created successfully");
        }
    }

    // Main render loop - runs at ~25 FPS (40ms)
    // This is separate from the video stream fps
    auto       last_render_time  = std::chrono::steady_clock::now();
    const auto target_frame_time = std::chrono::milliseconds(40);

    while(render_loop_running_.load(std::memory_order_acquire))
    {
        ZoneScopedN("RenderLoop::Iteration");
        FrameMark;

        std::unique_lock<std::mutex> lock(renderer_mutex_);

        // Wait for work or timeout
        render_cv_.wait_for(lock, target_frame_time,
                            [this]()
                            {
                                return !render_loop_running_.load(std::memory_order_acquire) ||
                                       render_requested_.load(std::memory_order_acquire);
                            });

        if(!render_loop_running_.load(std::memory_order_acquire))
            break;

        auto now = std::chrono::steady_clock::now();
        auto time_since_last_render =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render_time);

        // Always render at target frame rate, even if commands are pending
        // This ensures smooth output during heavy data loading
        bool should_render = time_since_last_render >= target_frame_time;

        // Process render commands, but limit processing time to maintain frame rate
        // Only process one command per frame to keep rendering smooth
        if(!render_commands_.empty() && !should_render)
        {
            ZoneScopedN("ProcessRenderCommand");
            auto command = std::move(render_commands_.front());
            render_commands_.pop();
            command();

            // Check if we should render now
            now = std::chrono::steady_clock::now();
            time_since_last_render =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - last_render_time);
            should_render = time_since_last_render >= target_frame_time;
        }

        // Render frame when it's time, regardless of pending commands
        if(renderer_ && should_render)
        {
            ZoneScopedN("RenderFrame");
            RCLCPP_DEBUG(this->get_logger(), "RenderLoop: rendering frame");
            auto frame = renderer_->Render();
            WriteData(frame.data(), frame.size());
            last_render_time = std::chrono::steady_clock::now();
        }
        else if(!should_render)
        {
            RCLCPP_DEBUG(this->get_logger(), "RenderLoop: skip render, time since last = %ldms",
                         time_since_last_render.count());
        }

        // Clear render request flag
        render_requested_.store(false, std::memory_order_release);

        // Publish notification to RTP node
        auto msg = std_msgs::msg::Empty();
        publisher_->publish(msg);

        // If there are still commands in the queue, immediately wake up for next iteration
        if(!render_commands_.empty())
        {
            render_requested_.store(true, std::memory_order_release);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Render thread stopped");
}

void Map_Visualizer::StopRenderLoop()
{
    render_loop_running_.store(false, std::memory_order_release);
    render_cv_.notify_all();
}

void Map_Visualizer::WriteData(void *data, size_t length)
{
    // Note: seqlock is incremented for even/odd checks. Could be changed to fetch-add
    // fetch-sub, but uint64_t is big enough to never cause issues.

    // Wait until subscriber isn't copying to its internal buffer
    WaitUntilSubscriberNotBusy();

    // Increment seq to mark writer-in-progress (make it odd).
    rtp_frame_->seq.fetch_add(1, std::memory_order_acq_rel);

    // Write payload
    rtp_frame_->data_len.store(length, std::memory_order_relaxed);
    std::memcpy(shm_data_ptr(rtp_frame_), data, length);

    // Ensure writes visible before completing write
    std::atomic_thread_fence(std::memory_order_release);

    // Increment seq (make it even, indicating stable)
    rtp_frame_->seq.fetch_add(1, std::memory_order_acq_rel);
}

#pragma region Camera
void Map_Visualizer::UpdateCameraPosition(
    const std::shared_ptr<interface_msgs::srv::CameraMovement::Request> request,
    std::shared_ptr<interface_msgs::srv::CameraMovement::Response>      response)
{
    // Capture all request values to avoid race conditions
    bool   update_zoom = request->update_zoom;
    double dzoom       = request->dzoom;
    bool   update_x    = request->update_x;
    double dx          = request->dx;
    bool   update_y    = request->update_y;
    double dy          = request->dy;
    bool   update_pan  = request->update_pan;
    double dpan        = request->dpan;

    EnqueueRenderCommand(
        [this, update_zoom, dzoom, update_x, dx, update_y, dy, update_pan, dpan]()
        {
            if(update_zoom)
            {
                int viewport_width, viewport_height;
                renderer_->GetCurrentReslution(viewport_width, viewport_height);
                float factor =
                    std::pow(static_cast<float>(viewport_height), static_cast<float>(dzoom));
                renderer_->UpdateCameraZoom(factor);
            }
            if(update_x)
            {
                float delta = static_cast<float>(dx);
                int   width;
                int   height;
                renderer_->GetCurrentReslution(width, height);
                delta *= width / renderer_->GetAbsZoom();
                renderer_->MoveCameraOnXAxis(delta);
            }
            if(update_y)
            {
                float delta = static_cast<float>(dy);
                int   width;
                int   height;
                renderer_->GetCurrentReslution(width, height);
                delta *= height / renderer_->GetAbsZoom();
                renderer_->MoveCameraOnYAxis(delta);
            }
            if(update_pan)
            {
                float delta = static_cast<float>(dpan);
                delta *= 90.f;
                renderer_->RotateCameraAroundZAxis(delta);
            }
        });

    response->success = true;
}

void Map_Visualizer::MoveCameraToRobot(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore = request;
    EnqueueRenderCommand([this]() { renderer_->MoveCameraToRobotPosition(); });

    response->success = true;
}

void Map_Visualizer::FollowRobot(
    const std::shared_ptr<interface_msgs::srv::FollowRobot::Request> request,
    std::shared_ptr<interface_msgs::srv::FollowRobot::Response>      response)
{
    uint8_t follow_type = request->follow_type;
    EnqueueRenderCommand(
        [this, follow_type]()
        {
            if(follow_type == interface_msgs::srv::FollowRobot::Request::OFF)
            {
                renderer_->FollowRobotOff();
            }
            else if(follow_type == interface_msgs::srv::FollowRobot::Request::POSITION)
            {
                renderer_->FollowRobotOn(false);
            }
            else if(follow_type ==
                    interface_msgs::srv::FollowRobot::Request::POSITION_AND_ORIENTATION)
            {
                renderer_->FollowRobotOn(true);
            }
        });
    response->success = true;
}

void Map_Visualizer::SetRobotPositionToCameraPosition(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore = request;

    // Get camera pose from render thread
    auto result = ExecuteOnRenderThread<std::tuple<glm::vec2, glm::vec4>>(
        [this]()
        {
            glm::vec2 camera_position;
            glm::vec4 camera_quaternion;
            renderer_->GetCameraPose(camera_position, camera_quaternion);
            return std::make_tuple(camera_position, camera_quaternion);
        });

    glm::vec2 camera_position   = std::get<0>(result);
    glm::vec4 camera_quaternion = std::get<1>(result);

    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose;
    robot_pose.pose.pose.position.x    = camera_position.x;
    robot_pose.pose.pose.position.y    = camera_position.y;
    robot_pose.pose.pose.position.z    = 0.0;
    robot_pose.pose.pose.orientation.x = camera_quaternion.x;
    robot_pose.pose.pose.orientation.y = camera_quaternion.y;
    robot_pose.pose.pose.orientation.z = camera_quaternion.z;
    robot_pose.pose.pose.orientation.w = camera_quaternion.w;
    robot_pose.header.stamp            = this->now();
    robot_pose.header.frame_id         = "map"; // Check whether this should be the current map name

    robot_initial_pose_->publish(robot_pose);

    response->success = true;
}

void Map_Visualizer::SetScaleUnitMeters(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    if(renderer_)
    {
        bool data = request->data;
        EnqueueRenderCommand([this, data]() { renderer_->SetScaleUnitMeters(data); });
    }
    response->success = true;
}

#pragma region Export
void Map_Visualizer::ExportMap(
    const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
    std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    std::string filename_base = request->data;

    // Execute entire export operation on render thread to ensure GL context affinity
    bool success = ExecuteOnRenderThread<bool>(
        [this, filename_base]()
        {
            // Save current resolution state
            int current_width, current_height;
            renderer_->GetCurrentReslution(current_width, current_height);

            int export_width, export_height;
            renderer_->GetExportResolution(export_width, export_height);

            // Center camera
            float center_x, center_y;
            renderer_->GetExportCenter(center_x, center_y);
            float camera_x, camera_y;
            renderer_->GetCameraPosition(camera_x, camera_y);
            bool follow_position, follow_orientation;
            renderer_->GetFollowRobotStatus(follow_position, follow_orientation);
            // Disable follow mode for export
            renderer_->FollowRobotOff();
            renderer_->SetAbsCameraPosition(center_x, center_y);

            // Target resolution: 20 pixels per meter (5cm per pixel)
            // This ensures each 5cm voxel renders as exactly 1 pixel for accurate representation
            float base_pixels_per_meter = 20.0f;

            // Calculate export dimensions at target resolution without zoom adjustment
            // Don't apply minimum size constraints as they would distort the scale and
            // cause voxels to render larger than intended (making separate pixels merge)
            int target_width = std::max(32, static_cast<int>(export_width * base_pixels_per_meter));
            int target_height =
                std::max(32, static_cast<int>(export_height * base_pixels_per_meter));

            float final_zoom = base_pixels_per_meter;

            float currentZoom     = renderer_->GetAbsZoom();
            float currentRotation = renderer_->GetAbsRotation();

            // IMPORTANT: Resize surface BEFORE setting zoom/rotation
            // The zoom calculation depends on window dimensions, so we must
            // resize first to ensure the projection matrix is correct
            renderer_->ResizeEGLSurface(target_width, target_height);

            renderer_->SetAbsZoom(final_zoom);
            renderer_->SetAbsRotation(0.0f);

            // render once
            auto frame = renderer_->Render(true, false, false);

            // save to disk
            std::error_code ec;
            if(!std::filesystem::exists(sd_card_dir_))
            {
                if(!std::filesystem::create_directories(sd_card_dir_, ec))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s",
                                 ec.message().c_str());
                    sd_card_dir_ = "";
                    return false;
                }
            }

            std::string           filename = filename_base + ".png";
            std::filesystem::path uri      = sd_card_dir_ / filename;
            stb_image_.Save(std::filesystem::absolute(uri).string().c_str(), &frame, target_width,
                            target_height, 3);

            // reset state
            renderer_->SetAbsCameraPosition(camera_x, camera_y);
            renderer_->SetAbsZoom(currentZoom);
            renderer_->SetAbsRotation(currentRotation);
            renderer_->ResizeEGLSurface(current_width, current_height);
            if(follow_position)
            {
                renderer_->FollowRobotOn(follow_orientation);
            }

            return true;
        });

    response->success = success;
}

#pragma region Markers
void Map_Visualizer::AddMarker(
    const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
    std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    std::string label = request->data;

    // Get camera position and add marker - both must happen on render thread
    auto result = ExecuteOnRenderThread<std::tuple<float, float>>(
        [this]()
        {
            float x, y;
            renderer_->GetCameraPosition(x, y);
            return std::make_tuple(x, y);
        });

    float x = std::get<0>(result);
    float y = std::get<1>(result);

    int index = CreateMarkerFile(label, x, y);

    // All marker data structure updates must happen on render thread
    EnqueueRenderCommand(
        [this, index, label, x, y]()
        {
            renderer_->AddMarker(index, label.c_str(), x, y);
            marker_map_.insert(std::pair<int, std::string>(index, label));
            active_marker_idx_ = index;
            RebuildMarkerUserIndex();
        });

    response->success = true;
}

void Map_Visualizer::RemoveMarker(
    const std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Response>      response)
{
    // Determine which marker index to remove - needs to read data structures
    // We need to do this synchronously to get the index before we can remove the file
    int index = ExecuteOnRenderThread<int>(
        [this, request]()
        {
            // index equals -1 when not provided
            int idx = 0;
            if(request->index == -1)
                idx = active_marker_idx_;
            else if(request->index < marker_user_idx_.size())
                idx = marker_user_idx_.at(request->index);
            return idx;
        });

    RemoveMarkerFile(index);

    // Update data structures on render thread
    EnqueueRenderCommand(
        [this, index]()
        {
            renderer_->RemoveMarker(index);
            marker_map_.erase(index);
            RebuildMarkerUserIndex();
        });

    response->success = true;
}

void Map_Visualizer::GetMarkers(
    const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
    std::shared_ptr<interface_msgs::srv::Listing::Response>      response)
{
    std::ignore = request;

    // Read marker data from render thread
    auto marker_data = ExecuteOnRenderThread<std::vector<std::string>>(
        [this]()
        {
            std::vector<std::string> data;
            for(auto &marker : marker_map_)
            {
                data.push_back(marker.second);
            }
            RebuildMarkerUserIndex();
            return data;
        });

    response->data = marker_data;
}

void Map_Visualizer::SetMarkerLabel(
    const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
    std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    std::string label = request->data;
    // Must execute on render thread to safely read active_marker_idx_
    // Also wait for disk write to complete before returning success
    ExecuteOnRenderThread<bool>(
        [this, label]()
        {
            UpdateMarkerLabel(active_marker_idx_, label);
            return true;
        });
    response->success = true;
}

void Map_Visualizer::SetMarkerLabelAt(
    const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
    std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response)
{
    std::string label = request->data;
    // Must execute on render thread to safely read marker_user_idx_
    // Also wait for disk write to complete before returning success
    ExecuteOnRenderThread<bool>(
        [this, request, label]()
        {
            int index = -1;
            if(request->index < marker_user_idx_.size())
                index = marker_user_idx_.at(request->index);

            UpdateMarkerLabel(index, label);
            return true;
        });
    response->success = true;
}

void Map_Visualizer::UpdateMarkerLabel(const int index, const std::string &label)
{
    // This function is now always called from render thread
    renderer_->UpdateMarkerLabel(index, label);

    if(marker_map_.count(index))
        marker_map_.at(index) = label;

    UpdateLabelInFile(index, label);
}

int Map_Visualizer::CreateMarkerFile(const std::string &label, float x, float y)
{
    int                index = GetNextIndex();
    std::ostringstream filename;

    // probably never going to exceed 1000 markers in a single file
    filename << std::setw(3) << std::setfill('0') << index << ".marker";
    std::filesystem::path file_path = view_path_ / filename.str();

    std::ofstream file(file_path);
    if(!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Could not create marker file %s",
                     file_path.c_str());
        return -1;
    }

    file << "label: " << label << "\n";
    file << "x: " << x << "\n";
    file << "y: " << y << "\n";
    file.close();

    return index;
}

void Map_Visualizer::LoadMarkersFromDisk()
{
    for(const auto &entry : std::filesystem::directory_iterator(view_path_))
    {
        if(entry.path().extension() == ".marker")
        {
            int           index = std::stoi(entry.path().stem().string());
            std::ifstream file(entry.path());
            std::string   line;
            MarkerFile    m;
            while(std::getline(file, line))
            {
                if(line.rfind("label:", 0) == 0)
                {
                    m.label = line.substr(7);
                }
                else if(line.rfind("x:", 0) == 0)
                {
                    m.x = std::stof(line.substr(3));
                }
                else if(line.rfind("y:", 0) == 0)
                {
                    m.y = std::stof(line.substr(3));
                }
            }

            file.close();

            // Send to renderer
            renderer_->AddMarker(index, m.label, m.x, m.y);
            marker_map_.insert(std::pair<int, std::string>(index, m.label));
        }
    }
    RebuildMarkerUserIndex();
}

void Map_Visualizer::RemoveMarkerFile(const int index)
{
    std::ostringstream filename;
    filename << std::setw(3) << std::setfill('0') << index << ".marker";
    std::filesystem::path file_path = view_path_ / filename.str();
    if(std::filesystem::exists(file_path))
    {
        std::filesystem::remove(file_path);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Marker file not found");
    }
}

int Map_Visualizer::GetNextIndex() const
{
    std::set<int> existing_indices;

    // Collect all marker indices
    for(const auto &entry : std::filesystem::directory_iterator(view_path_))
    {
        if(entry.path().extension() == ".marker")
        {
            try
            {
                int index = std::stoi(entry.path().stem().string());
                existing_indices.insert(index);
            }
            catch(...)
            {
            }
        }
    }

    // Find the first missing index starting from 0
    int expected = 0;
    for(int index : existing_indices)
    {
        if(index != expected)
            return expected;
        ++expected;
    }

    return expected;
}

void Map_Visualizer::UpdateLabelInFile(const int index, const std::string &label)
{
    std::ostringstream filename;
    filename << std::setw(3) << std::setfill('0') << index << ".marker";
    std::filesystem::path file_path = view_path_ / filename.str();
    if(!std::filesystem::exists(file_path))
        return;

    std::ifstream in(file_path);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string content = buffer.str();
    in.close();

    // Locate the label line
    const std::string prefix = "label:";
    std::size_t       pos    = content.find(prefix);
    if(pos == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "Label: not found in file");
        return;
    }

    // Find end of line
    std::size_t endLine = content.find('\n', pos);
    if(endLine == std::string::npos)
        endLine = content.size();

    // Replace the label line
    std::string newLine = "label: " + label;
    content.replace(pos, endLine - pos, newLine);

    // Write back to file
    std::ofstream out(file_path);
    out << content;
}

void Map_Visualizer::SetResolutionMode(
    const std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Request> request,
    std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Response>      response)
{
    uint8_t mode      = request->mode;
    response->success = true;

    if(mode == current_mode_)
        return;

    if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES)
    {
        RCLCPP_INFO(this->get_logger(), "Set resolution mode: high resolution.");
        float zoom_x = static_cast<float>(hr_frame_width_) / static_cast<float>(lr_frame_width_);
        float zoom_y = static_cast<float>(hr_frame_height_) / static_cast<float>(lr_frame_height_);
        float zoom   = std::min(zoom_x, zoom_y);

        EnqueueRenderCommand(
            [this, zoom]()
            {
                renderer_->UpdateCameraZoom(zoom);
                renderer_->ResizeEGLSurface(hr_frame_width_, hr_frame_height_);
            });
    }
    else if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::LOWRES)
    {
        RCLCPP_INFO(this->get_logger(), "Set resolution mode: low resolution.");
        float zoom_x = static_cast<float>(lr_frame_width_) / static_cast<float>(hr_frame_width_);
        float zoom_y = static_cast<float>(lr_frame_height_) / static_cast<float>(hr_frame_height_);
        float zoom   = std::min(zoom_x, zoom_y);

        EnqueueRenderCommand(
            [this, zoom]()
            {
                renderer_->UpdateCameraZoom(zoom);
                renderer_->ResizeEGLSurface(lr_frame_width_, lr_frame_height_);
            });
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Set resolution mode: invalid.");
        response->success = false;
        return;
    }

    // Signal RTP node
    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(mode);
    signal_resolution_->publish(msg);

    current_mode_ = mode;
}

void Map_Visualizer::ClearMap(const std_msgs::msg::Empty &msg)
{
    // Clear internals on render thread
    EnqueueRenderCommand(
        [this]()
        {
            marker_map_.clear();
            marker_user_idx_.clear();
            renderer_->RemoveAllMarkers();
            renderer_->SetPointCloudSources(nullptr, nullptr);
            renderer_->ClearTrajectory();
            renderer_->ClearMap();
        });
    // Map manager erases all content in map, so we don't have to erase files
}

void Map_Visualizer::UpdateRobotPose(const nav_msgs::msg::Odometry &msg)
{
    // Only update if we are viewing the localization floor
    if(view_path_ == localization_floor_path_)
    {
        // extract yaw from quaternion
        // requires taking into account all axes, even though we are only interested in yaw
        float yaw =
            atan2f(2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                           msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                   1.0f - 2.0f * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                                  msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));

        float pos_x = msg.pose.pose.position.x;
        float pos_y = -msg.pose.pose.position.y;

        EnqueueRenderCommand(
            [this, pos_x, pos_y, yaw]()
            {
                renderer_->UpdateRobotPosition(pos_x, pos_y);
                renderer_->UpdateRobotOrientation(yaw);
            });

        last_odometry_time_ = this->now();
    }
}

void Map_Visualizer::RebuildMarkerUserIndex()
{
    marker_user_idx_.clear();
    for(const auto &marker : marker_map_)
    {
        marker_user_idx_.push_back(marker.first);
    }
}
