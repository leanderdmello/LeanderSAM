// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "map_manager_node.hpp"

// C++ Standard Library
#include <chrono>

// ROS2 Core
#include "rclcpp/rclcpp.hpp"

// Project Headers
#include "slam_data_handler.hpp"
#include "states/idle_state.hpp"
#include "states/localizing_state.hpp"
#include "states/mapping_state.hpp"
#include "configuration.hpp"
#include "exceptions.h"

namespace
{

std::string SlamModeToString(State slam_mode_)
{
    switch(slam_mode_)
    {
    case State::Idle:
        return "Idle";
    case State::Mapping:
        return "Mapping";
    case State::Localizing:
        return "Localization";
    default:
        return "Unknown";
    }
}

State SlamModeFromMsg(uint8_t mode)
{
    switch(mode)
    {
    case slam_manager_msgs::msg::SlamMode::IDLE:
        return State::Idle;
    case slam_manager_msgs::msg::SlamMode::MAPPING:
        return State::Mapping;
    case slam_manager_msgs::msg::SlamMode::LOCALIZATION:
        return State::Localizing;
    default:
        throw std::invalid_argument("Unknown SLAM mode received in message");
    }
}
} // namespace

MapManagerNode::MapManagerNode() : rclcpp::Node("map_manager")
{
    Configuration config(*this);

    map_path_          = config.map_path;
    version_file_path_ = config.version_file_path;
    slam_mode_timeout  = config.slam_mode_timeout_ms;

    // Initialize state machine after paths are set
    current_state_     = std::make_unique<IdleState>(get_logger(), version_file_path_);
    map_save_interval_ = config.map_save_interval_ms;

    using std::placeholders::_1;
    using std::placeholders::_2;

    slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions slam_options;
    slam_options.callback_group = slam_callback_group_;
    control_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions control_options;
    control_options.callback_group = control_callback_group_;
    size_t qos_history_depth       = 1;

    clear_map_service_ = this->create_service<std_srvs::srv::Trigger>(
        config.srv_clear_map, std::bind(&MapManagerNode::ClearMap, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);

    get_software_versions_service_ = this->create_service<interface_msgs::srv::Listing>(
        config.srv_get_software_versions,
        std::bind(&MapManagerNode::GetSoftwareVersions, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);

    add_floor_service_ = this->create_service<interface_msgs::srv::SetString>(
        config.srv_add_floor, std::bind(&MapManagerNode::AddFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    delete_floor_service_ = this->create_service<interface_msgs::srv::ActionOnIndex>(
        config.srv_delete_floor, std::bind(&MapManagerNode::DeleteFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    clear_floor_service_ = this->create_service<std_srvs::srv::Trigger>(
        config.srv_clear_floor, std::bind(&MapManagerNode::ClearFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    clear_path_service_ = this->create_service<std_srvs::srv::Trigger>(
        config.srv_clear_path, std::bind(&MapManagerNode::ClearPath, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);

    get_floors_service_ = this->create_service<interface_msgs::srv::Listing>(
        config.srv_get_floors, std::bind(&MapManagerNode::GetFloors, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    get_mapping_floor_service_ = this->create_service<interface_msgs::srv::GetIndex>(
        config.srv_get_mapping_floor, std::bind(&MapManagerNode::GetMappingFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    get_view_floor_service_ = this->create_service<interface_msgs::srv::GetIndex>(
        config.srv_get_view_floor, std::bind(&MapManagerNode::GetViewFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    select_view_floor_service_ = this->create_service<interface_msgs::srv::ActionOnIndex>(
        config.srv_select_view_floor, std::bind(&MapManagerNode::SelectViewFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    select_mapping_floor_service_ = this->create_service<interface_msgs::srv::ActionOnIndex>(
        config.srv_select_mapping_floor,
        std::bind(&MapManagerNode::SelectMappingFloor, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);
    set_floor_name_service_ = this->create_service<interface_msgs::srv::UpdateStringAt>(
        config.srv_set_floor_name, std::bind(&MapManagerNode::SetFloorName, this, _1, _2),
        rmw_qos_profile_services_default, control_callback_group_);

    pointcloud_publisher_ = this->create_publisher<interface_msgs::msg::PointCloudAvailable>(
        config.topic_point_cloud_available, qos_history_depth);
    rclcpp::QoS mapping_pointcloud_publisher_qos = rclcpp::QoS(1);
    mapping_pointcloud_publisher_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    mapping_pointcloud_publisher_ =
        this->create_publisher<interface_msgs::msg::PointCloudAvailable>(
            config.topic_mapping_point_cloud_available, mapping_pointcloud_publisher_qos);
    trajectory_publisher_ = this->create_publisher<interface_msgs::msg::Trajectories>(
        config.topic_trajectories, qos_history_depth);
    clear_map_publisher_ = this->create_publisher<std_msgs::msg::Empty>("clear_map", 1);

    // Subscriptions for external mapping data
    receive_path_ = this->create_subscription<nav_msgs::msg::Path>(
        config.topic_trajectory, qos_history_depth,
        std::bind(&MapManagerNode::ReceivePath, this, _1), slam_options);

    receive_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        config.topic_global_map, qos_history_depth,
        std::bind(&MapManagerNode::ReceivePointCloud, this, _1), slam_options);

    slam_set_mode_ = this->create_client<slam_manager_msgs::srv::SetSlamMode>(
        config.srv_set_slam_mode, rmw_qos_profile_services_default, slam_callback_group_);

    rclcpp::QoS slam_mode_qos = rclcpp::QoS(qos_history_depth);
    slam_mode_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    slam_mode_subscription_ = this->create_subscription<slam_manager_msgs::msg::SlamMode>(
        config.topic_slam_mode, slam_mode_qos,
        std::bind(&MapManagerNode::OnSLAMModeChange, this, _1), slam_options);

    current_state_->OnEnter(map_path_ / current_map_name_, "", std::nullopt);
    savetimer_ =
        this->create_wall_timer(map_save_interval_, std::bind(&MapManagerNode::SaveData, this));
}

void MapManagerNode::GetSoftwareVersions(
    const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
    std::shared_ptr<interface_msgs::srv::Listing::Response>      response)
{
    try
    {
        current_state_->GetSoftwareVersions(response);
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::AddFloor(
    const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
    std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    try
    {
        ChangeMode(State::Mapping, request->data);
        current_state_->AddFloor(request, response);

        trajectory_       = std::nullopt;
        pointcloud_       = std::nullopt;
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::ClearMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    try
    {
        ChangeMode(State::Idle);
        trajectory_ = std::nullopt;
        pointcloud_ = std::nullopt;
        current_state_->ClearMap(response);

        // Signal visualizer to clear markers
        auto msg = std_msgs::msg::Empty();
        clear_map_publisher_->publish(msg);

        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::DeleteFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    try
    {
        if(current_state_->IsSLAMFloor(request->index))
        {
            ChangeMode(State::Idle);
        }
        current_state_->DeleteFloor(request, response);
        trajectory_       = std::nullopt;
        pointcloud_       = std::nullopt;
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::ClearFloor(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    try
    {
        auto view_floor = current_state_->GetViewFloorName();
        if(view_floor.has_value())
        {
            ChangeMode(State::Mapping, view_floor.value());
        }
        current_state_->ClearFloor(response);

        // Signal visualizer to clear markers
        auto msg = std_msgs::msg::Empty();
        clear_map_publisher_->publish(msg);

        trajectory_       = std::nullopt;
        pointcloud_       = std::nullopt;
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::ClearPath(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    try
    {
        current_state_->ClearPath(response);
        trajectory_               = std::nullopt;
        auto notification         = interface_msgs::msg::Trajectories();
        notification.trajectories = std::vector<nav_msgs::msg::Path>();
        trajectory_publisher_->publish(notification);
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::GetFloors(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                               std::shared_ptr<interface_msgs::srv::Listing::Response> response)
{
    try
    {
        current_state_->GetFloors(response);
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::GetMappingFloor(
    const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    try
    {
        current_state_->GetLocalizationFloor(request, response);
    }
    catch(const std::exception &e)
    {
        response->index = interface_msgs::srv::GetIndex::Response::NONE;
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::GetViewFloor(
    const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    try
    {
        current_state_->GetViewFloor(response);
    }
    catch(const std::exception &e)
    {
        response->index = interface_msgs::srv::GetIndex::Response::NONE;
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::SelectViewFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    try
    {
        current_state_->SelectViewFloor(request, response);
        auto view_floor = current_state_->GetViewFloorName();

        if(!current_state_->StreamMapLive() && view_floor.has_value())
        {
            std::filesystem::path view_path  = map_path_ / current_map_name_ / view_floor.value();
            const auto            pointcloud = slam_data_handler_->GetPointCloud(view_path);

            if(pointcloud.has_value())
            {
                slam_data_handler_->SharePointCloud(pointcloud.value());
                auto notification       = interface_msgs::msg::PointCloudAvailable();
                notification.floor_path = view_path.string();
                pointcloud_publisher_->publish(notification);
                RCLCPP_INFO(this->get_logger(), "Sending map %s.", view_path.c_str());
            }
            else
            {
                auto msg = std_msgs::msg::Empty();
                clear_map_publisher_->publish(msg);
            }
            auto notification      = interface_msgs::msg::Trajectories();
            auto past_trajectories = slam_data_handler_->GetPathClouds(view_path);
            if(!past_trajectories.empty())
            {
                notification.trajectories = past_trajectories;
            }
            else
            {
                notification.trajectories = std::vector<nav_msgs::msg::Path>();
            }
            const auto path = slam_data_handler_->GetPathCloud(view_path);
            if(path.has_value())
            {
                notification.trajectories.push_back(path.value());
            }
            trajectory_publisher_->publish(notification);
            RCLCPP_INFO(this->get_logger(), "Sending %ld paths.", notification.trajectories.size());
        }
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::SelectMappingFloor(
    const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    try
    {
        std::string floor_name = current_state_->GetFloorName(request->index);

        ChangeMode(State::Localizing, floor_name);
        current_state_->SelectLocalizationFloor(request, response);
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::SetFloorName(
    const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
    std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response)
{
    try
    {
        bool is_slam_floor = current_state_->IsSLAMFloor(request->index);
        current_state_->SetFloorName(request, response);
        if(is_slam_floor)
        {
            current_state_->ChangeSLAMFloorName(request->data);
        }
        response->success = true;
    }
    catch(const std::exception &e)
    {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
}

void MapManagerNode::ReceivePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
    if(current_state_->AcceptMapLive())
    {
        pointcloud_ = message;
        if(current_state_->StreamMapLive())
        {
            auto pointcloud = slam_data_handler_->ConvertRosMessage(message);
            if(current_state_->OldMergeMap().has_value())
            {
                *pointcloud += *(current_state_->OldMergeMap().value());
            }

            if(current_state_->GetSLAMFloorPath().has_value())
            {
                slam_data_handler_->SharePointCloud(pointcloud);
                auto notification       = interface_msgs::msg::PointCloudAvailable();
                notification.floor_path = current_state_->GetSLAMFloorPath().value().string();
                pointcloud_publisher_->publish(notification);
                RCLCPP_DEBUG(this->get_logger(), "Sending pointcloud");
            }
        }
    }
}

void MapManagerNode::ReceivePath(const nav_msgs::msg::Path::SharedPtr message)
{
    if(current_state_->AcceptPathLive())
    {
        trajectory_     = message;
        auto view_floor = current_state_->GetViewFloorName();
        if(current_state_->StreamPathLive() && view_floor.has_value())
        {
            auto                  notification = interface_msgs::msg::Trajectories();
            std::filesystem::path view_path    = map_path_ / current_map_name_ / view_floor.value();
            notification.trajectories          = slam_data_handler_->GetPathClouds(view_path);
            notification.trajectories.push_back(*message);
            trajectory_publisher_->publish(notification);
            RCLCPP_DEBUG(this->get_logger(), "Received path, sending: %ld",
                         notification.trajectories.size());
        }
    }
}

void MapManagerNode::SaveData()
{
    // Only save if we have a SLAM floor path in the current state
    if(!current_state_->GetSLAMFloorPath().has_value())
    {
        return;
    }
    auto save_map_dir = current_state_->GetSLAMFloorPath().value();
    slam_data_handler_->Backup(save_map_dir);
    if(pointcloud_.has_value())
    {
        auto pointcloud_obj = slam_data_handler_->ConvertRosMessage(pointcloud_.value());
        if(current_state_->OldMergeMap().has_value())
        {
            *(pointcloud_obj) += *(current_state_->OldMergeMap().value());
        }
        slam_data_handler_->SavePointCloud(pointcloud_obj, save_map_dir);
    }
    if(trajectory_.has_value())
    {
        slam_data_handler_->SavePath(trajectory_.value(), save_map_dir);
    }
    trajectory_ = std::nullopt;
    pointcloud_ = std::nullopt;
}

void MapManagerNode::ChangeMode(State new_mode, const std::string &floor_name)
{
    changing_slam_mode_ = true;
    ChangeSLAMMode(slam_manager_msgs::msg::SlamMode::IDLE);
    SaveData();
    std::optional<std::string> view_floor = current_state_->GetViewFloorName();
    current_state_->OnExit();

    std::filesystem::path mapping_floor_path         = map_path_ / current_map_name_ / floor_name;
    auto                  mapping_floor_notification = interface_msgs::msg::PointCloudAvailable();
    mapping_floor_notification.floor_path            = mapping_floor_path.string();

    try
    {
        switch(new_mode)
        {
        case State::Idle:
        {
            current_state_ = std::make_unique<IdleState>(get_logger(), version_file_path_);
            // leave floor_path empty to signal no floor is selected
            mapping_floor_notification.floor_path = "";
            break;
        }
        case State::Mapping:
        {
            if(floor_name.empty())
            {
                throw std::runtime_error("empty floor not allowed in mapping mode");
            }
            ChangeSLAMMode(slam_manager_msgs::msg::SlamMode::MAPPING);
            current_state_ = std::make_unique<MappingState>(get_logger(), version_file_path_);
            break;
        }
        case State::Localizing:
        {
            ChangeSLAMMode(slam_manager_msgs::msg::SlamMode::LOCALIZATION);
            current_state_ = std::make_unique<LocalizingState>(get_logger(), version_file_path_);
            break;
        }
        }
    }
    catch(const FailedToChangeSlamStateException &e)
    {
        // SLAM will be in IDLE mode if changing fails here, so set state to IdleState
        // Simply call this method again to ensure state transition is done completely, then rethrow
        ChangeMode(State::Idle);
        std::rethrow_exception(std::current_exception());
    }
    current_state_->OnEnter(map_path_ / current_map_name_, floor_name, view_floor);
    // Start a new map only when we have a SLAM map in the current state
    if(current_state_->GetSLAMFloorPath().has_value())
    {
        slam_data_handler_->StartPath(current_state_->GetSLAMFloorPath().value());
    }
    RCLCPP_INFO(this->get_logger(),
                "SLAM mode changed from %s to %s with view floor %s and active %s.",
                SlamModeToString(slam_mode_).c_str(), SlamModeToString(new_mode).c_str(),
                view_floor.has_value() ? view_floor.value().c_str() : "", floor_name.c_str());
    slam_mode_ = new_mode;
    mapping_pointcloud_publisher_->publish(mapping_floor_notification);
    changing_slam_mode_ = false;
}

void MapManagerNode::ChangeSLAMMode(uint8_t new_mode)
{
    auto request = std::make_shared<slam_manager_msgs::srv::SetSlamMode::Request>();
    slam_manager_msgs::msg::SlamMode mode;
    mode.mode     = new_mode;
    request->mode = mode;
    auto result   = slam_set_mode_->async_send_request(request);
    auto status   = result.wait_for(slam_mode_timeout);
    if(status != std::future_status::ready || !result.get()->success)
    {
        throw FailedToChangeSlamStateException(new_mode);
    }
}

void MapManagerNode::OnSLAMModeChange(
    const std::shared_ptr<slam_manager_msgs::msg::SlamMode> message)
{
    State incoming_mode = SlamModeFromMsg(message->mode);
    if(incoming_mode != slam_mode_ && !changing_slam_mode_)
    {
        RCLCPP_WARN(this->get_logger(), "SLAM mode %s is different than expected! Changing to %s.",
                    SlamModeToString(incoming_mode).c_str(), SlamModeToString(slam_mode_).c_str());
        ChangeMode(slam_mode_, current_state_->GetSLAMFloorName());
    }
}
