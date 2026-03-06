// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <tracy/Tracy.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "../helpers/shared_memory.h"
#include "pc_renderer/renderer.h"
#include "pc_renderer/stb_wrapper.h"
#include "pc_renderer/point_cloud.h"

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <functional>
#include <future>

#include "interface_msgs/msg/point_cloud_available.hpp"
#include "interface_msgs/msg/trajectories.hpp"
#include "interface_msgs/srv/action_on_optional_index.hpp"
#include "interface_msgs/srv/camera_movement.hpp"
#include "interface_msgs/srv/follow_robot.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/update_string_at.hpp"
#include "interface_msgs/srv/set_video_resolution_mode.hpp"
#include "interface_msgs/msg/point_cloud_available.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Map_Visualizer : public rclcpp::Node
{
public:
    // Simple struct that represents a placed marker
    struct MarkerFile
    {
        std::string label;
        float       x;
        float       y;
    };

    Map_Visualizer(int width, int height);
    Map_Visualizer();
    ~Map_Visualizer();

    void CleanupGLResources();

    // Main render loop - runs in dedicated thread
    void RenderLoop();
    void StopRenderLoop();

private:
    // Get map from map manager
    void ReadPointCloudData(const interface_msgs::msg::PointCloudAvailable &msg);
    void ReadTrajectoryData(const interface_msgs::msg::Trajectories &msg);
    bool TryReadAndCopyMap(std::vector<float> &data);

    void ProcessLocalizationPath(const interface_msgs::msg::PointCloudAvailable &msg);

    void RenderMap();

    // Simple helper to wait until subscriber is not busy.
    // spins with short sleep to avoid busy-waiting CPU burn.
    void WaitUntilSubscriberNotBusy();
    void WriteData(void *data, size_t length);

    // Markers methods
    void AddMarker(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                   std::shared_ptr<interface_msgs::srv::SetString::Response>      response);

    void RemoveMarker(
        const std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Request> request,
        std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Response>      response);

    void GetMarkers(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                    std::shared_ptr<interface_msgs::srv::Listing::Response>      response);

    void SetMarkerLabel(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                        std::shared_ptr<interface_msgs::srv::SetString::Response>      response);

    void SetMarkerLabelAt(
        const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
        std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response);

    int  CreateMarkerFile(const std::string &label, float x, float y);
    void GetMarkerNames(std::vector<std::string> &MarkerNames);
    int  GetNextIndex() const;
    void LoadMarkersFromDisk();
    void RemoveMarkerFile(const int index);
    void UpdateMarkerLabel(const int index, const std::string &label);
    void UpdateLabelInFile(const int index, const std::string &label);
    void ClearMap(const std_msgs::msg::Empty &msg);

    void RebuildMarkerUserIndex();

    // Camera movement methods
    void UpdateCameraPosition(
        const std::shared_ptr<interface_msgs::srv::CameraMovement::Request> request,
        std::shared_ptr<interface_msgs::srv::CameraMovement::Response>      response);
    void MoveCameraToRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response>      response);
    void FollowRobot(const std::shared_ptr<interface_msgs::srv::FollowRobot::Request> request,
                     std::shared_ptr<interface_msgs::srv::FollowRobot::Response>      response);
    void SetRobotPositionToCameraPosition(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response>      response);

    // Scale unit
    void SetScaleUnitMeters(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response>      response);

    // Video and rendering resolution
    void SetResolutionMode(
        const std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Request> request,
        std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Response>      response);

    // Export map
    void ExportMap(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                   std::shared_ptr<interface_msgs::srv::SetString::Response>      response);

    // Robot odometry handling
    void UpdateRobotPose(const nav_msgs::msg::Odometry &msg);

    // Helper method to signal render thread
    void RequestRender();

    // Helper method to enqueue a renderer command to be executed on the render thread
    void EnqueueRenderCommand(std::function<void()> command);

    // Helper method to execute a renderer command synchronously on the render thread
    template <typename R>
    R ExecuteOnRenderThread(std::function<R()> command);

    // Thread synchronization primitives for safe rendering in multithreaded environment
    mutable std::mutex      renderer_mutex_;            // Protects renderer state and command queue
    std::condition_variable render_cv_;                 // Signals render thread when updates arrive
    std::atomic<bool>       render_requested_{false};   // Flag: render requested by data callbacks
    std::atomic<bool>       render_loop_running_{true}; // Flag: render loop should continue
    std::queue<std::function<void()>>
        render_commands_; // Queue of commands to execute on render thread

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                        publisher_;
    rclcpp::Subscription<interface_msgs::msg::PointCloudAvailable>::SharedPtr pc_subscription_;
    rclcpp::Subscription<interface_msgs::msg::Trajectories>::SharedPtr        tr_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr    clear_map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odometry_subscription_;
    rclcpp::TimerBase::SharedPtr                             odometry_watchdog_timer_;
    rclcpp::Time                                             last_odometry_time_;
    rclcpp::Subscription<interface_msgs::msg::PointCloudAvailable>::SharedPtr
        pc_localization_subscription_;

    // Callback groups for concurrent subscription handling
    rclcpp::CallbackGroup::SharedPtr pc_callback_group_;
    rclcpp::CallbackGroup::SharedPtr pc_localization_callback_group_;
    rclcpp::CallbackGroup::SharedPtr tr_callback_group_;
    rclcpp::CallbackGroup::SharedPtr clear_map_callback_group_;
    rclcpp::CallbackGroup::SharedPtr robot_odometry_callback_group_;

    rclcpp::Service<interface_msgs::srv::SetVideoResolutionMode>::SharedPtr     resolution_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                         signal_resolution_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_initial_pose_;

    // shared memory
    int        rtp_fd_;
    ShmLayout *rtp_frame_;

    // shared memory
    int        map_fd_;
    ShmLayout *map_frame_;

    // marker path and storage
    std::filesystem::path view_path_;
    // localization floor path for comparing with view floor path
    std::filesystem::path localization_floor_path_;
    // internal map that maps the file system indexes to marker names
    // Protected by renderer_mutex_ - only access from render thread or with lock
    std::map<int, std::string> marker_map_;
    // vector that stores the internal index of the list of maps sent to the user.
    // Protected by renderer_mutex_ - only access from render thread or with lock
    std::vector<int> marker_user_idx_;
    // index of the currently selected marker
    // Protected by renderer_mutex_ - only access from render thread or with lock
    int active_marker_idx_{0};

    uint8_t current_mode_{interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES};
    int     hr_frame_width_;
    int     hr_frame_height_;

    int                   lr_frame_width_;
    int                   lr_frame_height_;
    std::string           sd_card_path_;
    std::filesystem::path sd_card_dir_;

    // Gstreamer
    int frame_size_;
    int frame_count_ = 0;

    std::unique_ptr<Map_Renderer> renderer_;
    StbImage                      stb_image_;

    // Point cloud data holders for split preprocessing + GPU upload
    std::unique_ptr<PointCloud> map_;
    std::unique_ptr<PointCloud> trajectory_;

    // Camera movement
    rclcpp::Service<interface_msgs::srv::CameraMovement>::SharedPtr camera_movement_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              move_camera_to_robot_;
    rclcpp::Service<interface_msgs::srv::FollowRobot>::SharedPtr    follow_robot_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_robot_position_to_camera_position_;

    // Markers
    rclcpp::Service<interface_msgs::srv::SetString>::SharedPtr             add_marker_;
    rclcpp::Service<interface_msgs::srv::ActionOnOptionalIndex>::SharedPtr remove_marker_;
    rclcpp::Service<interface_msgs::srv::Listing>::SharedPtr               get_markers_;
    rclcpp::Service<interface_msgs::srv::SetString>::SharedPtr             set_marker_label_;
    rclcpp::Service<interface_msgs::srv::UpdateStringAt>::SharedPtr        set_marker_label_at_;

    // Scale unit
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr scale_unit_meters_;

    // Exports
    rclcpp::Service<interface_msgs::srv::SetString>::SharedPtr export_map_;
};
