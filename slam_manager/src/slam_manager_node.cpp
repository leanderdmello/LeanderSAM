// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "slam_manager_node.hpp"
#include <rclcpp_avular/helper_service.hpp>

#include <algorithm>

void SlamModeConfig::Configure(SlamManager *node, const std::uint8_t &slam_mode)
{
    std::string mode_string = SlamModeToString(slam_mode);
    // Get parameters
    std::vector<std::string> transitions;
    node->GetParameter(fmt::format("states.{}.transitions_to", mode_string), &transitions);

    for(const auto &transition : transitions)
    {
        this->transitions.push_back(StringToSlamMode(transition).mode);
    }
}

SlamManager::SlamManager(rclcpp::NodeOptions options) : Node("slam_manager", options)
{
    // Create configurations for each slam mode.
    config_map_ = {
        {slam_manager_msgs::msg::SlamMode::IDLE, std::make_shared<SlamModeConfig>()},
        {slam_manager_msgs::msg::SlamMode::MAPPING, std::make_shared<SlamModeConfig>()},
        {slam_manager_msgs::msg::SlamMode::LOCALIZATION, std::make_shared<SlamModeConfig>()}};

    // Configure each slam mode.
    for(auto &[mode, config] : config_map_)
    {
        config->Configure(this, mode);
    }

    // Other parameters
    std::string mode_out_topic, set_mode_srv_topic, set_state_client_topic;
    GetParameter("topics.out.slam_mode", &mode_out_topic);
    GetParameter("services.server.set_slam_mode", &set_mode_srv_topic);
    GetParameter("services.client.set_state", &set_state_client_topic);
    double timeout_sec_;
    GetParameter("timeout_threshold", &timeout_sec_);
    timeout_threshold_ = std::chrono::milliseconds(static_cast<int>(timeout_sec_ * 1000));

    // Setup publishers
    pub_mode_ = this->create_publisher<slam_manager_msgs::msg::SlamMode>(
        mode_out_topic, rclcpp_avular::qos::Latching());

    // setup callback groups
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Setup services
    srv_set_mode_ = this->create_service<slam_manager_msgs::srv::SetSlamMode>(
        set_mode_srv_topic,
        [this](std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Request> const  request,
               std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Response> const response)
        {
            try
            {
                SetSlamModeRequest(request, response);
            }
            catch(std::invalid_argument &ex)
            {
                response->success = false;
                response->message = ex.what();
            }
            if(response->success)
            {
                RCLCPP_INFO(get_logger(), response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(get_logger(), response->message.c_str());
            }
        });

    // Setup Clients
    client_set_state_ = this->create_client<autonomy_msgs::srv::SetState>(
        set_state_client_topic, rmw_qos_profile_services_default, client_cb_group_);

    // Publish default state on startup. It is latching so will be available to nodes that start
    // later.
    pub_mode_->publish(mode_);
}

void SlamManager::SetSlamModeRequest(
    std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Request> const request,
    std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Response>      response)
{
    RCLCPP_INFO(get_logger(),
                fmt::format("Request received to change current mode `{}` to `{}`",
                            SlamModeToString(mode_.mode), SlamModeToString(request->mode.mode))
                    .c_str());

    if(request->mode == mode_)
    {
        response->success = true;
        response->message = fmt::format("Target control mode `{}` is already active.",
                                        SlamModeToString(mode_.mode));
        return;
    }
    else if(request->mode.mode == slam_manager_msgs::msg::SlamMode::IDLE)
    {
        SetSlamMode(request->mode);
        response->success = true;
        response->message = "Slam mode reset, system in idle.";
        return;
    }
    else if(IsValidTransition(request->mode.mode))
    {
        SetSlamMode(request->mode);
        if(mode_ == request->mode)
        {
            response->success = true;
            response->message =
                fmt::format("Control mode set to {}.", SlamModeToString(mode_.mode));
        }
        else
        {
            response->success = false;
            response->message = fmt::format("Control mode could not be set, mode still in {}.",
                                            SlamModeToString(mode_.mode));
        }
        return;
    }
    else
    {
        response->message = fmt::format("Something went wrong. Requested control mode with id `{}` "
                                        "is not one of the accepted slam modes for mode "
                                        "{}.",
                                        request->mode.mode, mode_.mode);
        response->success = false;
        return;
    }
}


void SlamManager::SetSlamMode(slam_manager_msgs::msg::SlamMode const mode)
{
    auto request   = std::make_shared<autonomy_msgs::srv::SetState::Request>();
    request->state = SlamModeToString(mode.mode);

    auto response = rclcpp_avular::QueryService(client_set_state_, request, timeout_threshold_);
    if(!response)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Could not contact lifecycle manager to set lifecycle state");
        return;
    }
    if(!response.value()->success)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            fmt::format("Could not enable lifecycle state: {}", response.value()->message).c_str());
        return;
    }

    RCLCPP_INFO(get_logger(),
                fmt::format("Set lifecycle state to {}", SlamModeToString(mode.mode)).c_str());

    mode_previous_ = mode_;
    mode_          = mode;
    pub_mode_->publish(mode_);

    return;
}

bool SlamManager::IsValidTransition(std::uint8_t const mode)
{
    // Transition from None is always allowed
    if(mode_.mode == slam_manager_msgs::msg::SlamMode::IDLE)
    {
        return true;
    }

    auto valid_transitions = config_map_[mode_.mode]->transitions;
    return (std::find(valid_transitions.begin(), valid_transitions.end(), mode) !=
            valid_transitions.end());
}

RCLCPP_COMPONENTS_REGISTER_NODE(SlamManager)
