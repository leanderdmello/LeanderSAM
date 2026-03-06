// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <fmt/format.h>

#include <autonomy_msgs/srv/set_state.hpp>
#include <autonomy_msgs/srv/get_state.hpp>
#include "slam_manager_msgs/msg/slam_mode.hpp"
#include "slam_manager_msgs/srv/set_slam_mode.hpp"
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_avular/node.hpp>


inline std::string SlamModeToString(std::uint8_t const mode)
{
  switch(mode)
  {
    case slam_manager_msgs::msg::SlamMode::IDLE:
    {
      return "idle";
    }
    case slam_manager_msgs::msg::SlamMode::MAPPING:
    {
      return "mapping";
    }
    case slam_manager_msgs::msg::SlamMode::LOCALIZATION:
    {
      return "localization";
    }
    default:
    {
      throw std::invalid_argument(fmt::format("Slam mode identifier {} not recognized.", mode));
    }
  }
}

inline slam_manager_msgs::msg::SlamMode StringToSlamMode(std::string const& mode)
{
  slam_manager_msgs::msg::SlamMode msg;
  if(mode == "idle")
  {
    msg.mode = slam_manager_msgs::msg::SlamMode::IDLE;
  }
  else if(mode == "mapping")
  {
    msg.mode = slam_manager_msgs::msg::SlamMode::MAPPING;
  }
  else if(mode == "localization")
  {
    msg.mode = slam_manager_msgs::msg::SlamMode::LOCALIZATION;
  }
  else
  {
    throw std::invalid_argument(fmt::format("Slam mode string {} not recognized.", mode));
  }
  return msg;
}

class SlamManager;

struct SlamModeConfig
{
  virtual void Configure(SlamManager* node, const std::uint8_t& slam_mode);
  std::string topic;
  std::vector<std::uint8_t> transitions;
};

class SlamManager : public rclcpp_avular::Node
{
public:
  SlamManager(rclcpp::NodeOptions options);

private:
  // Functions
  
  /**
   * @brief Process an incoming SetControlMode request
   *
   * @param request incoming request
   * @param response response to be returned
   */
  void SetSlamModeRequest(std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Request> const request,
                             std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Response> const response);

  /**
   * @brief Set the lifecycle state as defined in the mode and update the previous slam mode. No checks are included.
   *
   * @param mode The desired mode/state
   *
   * @note Parameter not reference to avoid issues with mode_previous_.
   */
  void SetSlamMode(slam_manager_msgs::msg::SlamMode const mode);
   
  /**
   * @brief Checks whether a transition to the desired mode is allowed from the current control mode
   *
   * @param mode desired control mode
   * @returns true if valid
   * @returns false if invalid
   */
  bool IsValidTransition(std::uint8_t const mode);

  // Interfaces
  rclcpp::Publisher<slam_manager_msgs::msg::SlamMode>::SharedPtr pub_mode_;
  rclcpp::Service<slam_manager_msgs::srv::SetSlamMode>::SharedPtr srv_set_mode_;
  rclcpp::Client<autonomy_msgs::srv::SetState>::SharedPtr client_set_state_;
  rclcpp::TimerBase::SharedPtr timer_stale_input_;

  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  // Parameters
  std::chrono::milliseconds timeout_threshold_;

  // Variables
  std::map<std::uint8_t, std::shared_ptr<SlamModeConfig>> config_map_;  // Configurations
  bool is_stale_ = false;

  // Current and previous slam modes
  slam_manager_msgs::msg::SlamMode mode_;
  slam_manager_msgs::msg::SlamMode mode_previous_;

  // Friends
  friend void SlamModeConfig::Configure(SlamManager*, const std::uint8_t&);
};