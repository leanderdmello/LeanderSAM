// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "interface_msgs/srv/set_string.hpp"
#include "interface_msgs/srv/listing.hpp"
#include "interface_msgs/msg/robot_status.hpp"
#include "interface_msgs/srv/camera_movement.hpp"
#include "interface_msgs/srv/follow_robot.hpp"
#include "interface_msgs/srv/action_on_index.hpp"
#include "interface_msgs/srv/get_index.hpp"
#include "interface_msgs/srv/update_string_at.hpp"
#include "interface_msgs/srv/action_on_optional_index.hpp"
#include "interface_msgs/srv/set_video_resolution_mode.hpp"
#include "interface_msgs/srv/set_video_stream_address.hpp"
#include "interface_msgs/msg/point_cloud_available.hpp"
#include "interface_msgs/msg/trajectories.hpp"
#include "slam_manager_msgs/srv/set_slam_mode.hpp"
#include "slam_manager_msgs/msg/slam_mode.hpp"

std::shared_ptr<rclcpp::Node> node;

void ExportMap(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
               std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Export Map: %s.", request->data.c_str());
}

void ClearMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore       = request;
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Clear Map.");
}

void ModifyCamera(const std::shared_ptr<interface_msgs::srv::CameraMovement::Request> request,
                  std::shared_ptr<interface_msgs::srv::CameraMovement::Response>      response)
{
    std::stringstream stream;
    stream << "Modify camera";
    if(request->update_zoom)
    {
        stream << " zoom " << request->dzoom;
    }
    if(request->update_x)
    {
        stream << " xTranslation " << request->dx;
    }
    if(request->update_y)
    {
        stream << " yTranslation " << request->dy;
    }
    if(request->update_pan)
    {
        stream << " pan " << request->dpan;
    }
    if(request->update_tilt)
    {
        stream << " tilt " << request->dtilt;
    }
    RCLCPP_INFO(node->get_logger(), stream.str().c_str());

    response->success = true;
}

void MoveCameraToRobot(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore       = request;
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Move camera to robot.");
}

void FollowRobot(const std::shared_ptr<interface_msgs::srv::FollowRobot::Request> request,
                 std::shared_ptr<interface_msgs::srv::FollowRobot::Response>      response)
{
    response->success   = true;
    uint8_t follow_type = request->follow_type;
    if(follow_type == interface_msgs::srv::FollowRobot::Request::OFF)
    {
        RCLCPP_INFO(node->get_logger(), "Follow robot: off.");
    }
    else if(follow_type == interface_msgs::srv::FollowRobot::Request::POSITION)
    {
        RCLCPP_INFO(node->get_logger(), "Follow robot: position.");
    }
    else if(follow_type == interface_msgs::srv::FollowRobot::Request::POSITION_AND_ORIENTATION)
    {
        RCLCPP_INFO(node->get_logger(), "Follow robot: position and orientation.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Follow robot: invalid.");
    }
}

void SetRobotLocationToCamera(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore       = request;
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set robot location to camera.");
}

void GetSoftwareVersions(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                         std::shared_ptr<interface_msgs::srv::Listing::Response>      response)
{
    std::ignore    = request;
    response->data = std::vector<std::string>{"robot: 1.4.5", "SW: 2.4.5"};
    RCLCPP_INFO(node->get_logger(), "Get software versions.");
}

void AddFloor(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
              std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Add floor: %s.", request->data.c_str());
}

void DeleteFloor(const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
                 std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Delete floor: %d.", request->index);
}

void ClearFloor(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore       = request;
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Clear floor.");
}

void GetFloors(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
               std::shared_ptr<interface_msgs::srv::Listing::Response>      response)
{
    std::ignore    = request;
    response->data = std::vector<std::string>{"floor1", "floor2"};
    RCLCPP_INFO(node->get_logger(), "Get floors.");
}

void GetMappingFloor(const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
                     std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    std::ignore     = request;
    response->index = 1;
    RCLCPP_INFO(node->get_logger(), "Get mapping floor.");
}

void GetViewFloor(const std::shared_ptr<interface_msgs::srv::GetIndex::Request> request,
                  std::shared_ptr<interface_msgs::srv::GetIndex::Response>      response)
{
    std::ignore     = request;
    response->index = 1;
    RCLCPP_INFO(node->get_logger(), "Get view floor.");
}

void SelectViewFloor(const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
                     std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    std::ignore = response;
    RCLCPP_INFO(node->get_logger(), "Select view floor: %d.", request->index);
}

void SelectMappingFloor(const std::shared_ptr<interface_msgs::srv::ActionOnIndex::Request> request,
                        std::shared_ptr<interface_msgs::srv::ActionOnIndex::Response>      response)
{
    std::ignore = response;
    RCLCPP_INFO(node->get_logger(), "Select mapping floor: %d.", request->index);
}

void SetFloorName(const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
                  std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set floor name: %s %d.", request->data.c_str(),
                request->index);
}

void AddMarker(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
               std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Add marker: %s.", request->data.c_str());
}

void RemoveMarker(
    const std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Request> request,
    std::shared_ptr<interface_msgs::srv::ActionOnOptionalIndex::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Remove marker: %d.", request->index);
}

void GetMarkers(const std::shared_ptr<interface_msgs::srv::Listing::Request> request,
                std::shared_ptr<interface_msgs::srv::Listing::Response>      response)
{
    std::ignore    = request;
    response->data = std::vector<std::string>{"Marker 1", "Marker 2"};
    RCLCPP_INFO(node->get_logger(), "Get markers.");
}

void SetMarkerLabel(const std::shared_ptr<interface_msgs::srv::SetString::Request> request,
                    std::shared_ptr<interface_msgs::srv::SetString::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set marker label: %s.", request->data.c_str());
}

void SetMarkerLabelAt(const std::shared_ptr<interface_msgs::srv::UpdateStringAt::Request> request,
                      std::shared_ptr<interface_msgs::srv::UpdateStringAt::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set marker label: %s %d.", request->data.c_str(),
                request->index);
}

void SetScaleUnitMeters(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set scale unit meters: %d.", request->data);
}

void SetVideoResolutionMode(
    const std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Request> request,
    std::shared_ptr<interface_msgs::srv::SetVideoResolutionMode::Response>      response)
{
    response->success = true;
    uint8_t mode      = request->mode;
    if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::HIGHRES)
    {
        RCLCPP_INFO(node->get_logger(), "Set video resolution mode: high resolution.");
    }
    else if(mode == interface_msgs::srv::SetVideoResolutionMode::Request::LOWRES)
    {
        RCLCPP_INFO(node->get_logger(), "Set video resolution mode: low resolution.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Set video resolution mode: invalid.");
    }
}

void SetVideoStreamAddress(
    const std::shared_ptr<interface_msgs::srv::SetVideoStreamAddress::Request> request,
    std::shared_ptr<interface_msgs::srv::SetVideoStreamAddress::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set video stream address: %s %d.", request->ip_address.c_str(),
                request->port);
}

void SetVideoStream(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Set video stream: %d.", request->data);
}

void ClearPath(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response>      response)
{
    std::ignore       = request;
    response->success = true;
    RCLCPP_INFO(node->get_logger(), "Clear path.");
}

void RobotStatusChanged(const interface_msgs::msg::RobotStatus::SharedPtr message)
{
    std::stringstream stream;
    stream << "Robot status change " << message->robot_is_moving;
    if((message->hardware_flags & interface_msgs::msg::RobotStatus::LONG_FLIPPERS) > 0)
    {
        stream << " Long flippers";
    }
    if((message->hardware_flags & interface_msgs::msg::RobotStatus::SHORT_FLIPPERS) > 0)
    {
        stream << " Short flippers";
    }
    if((message->hardware_flags & interface_msgs::msg::RobotStatus::CLIMBING_AID) > 0)
    {
        stream << " Climbing aid";
    }
    RCLCPP_INFO(node->get_logger(), stream.str().c_str());
}

void PointCloudAvailable(const interface_msgs::msg::PointCloudAvailable::SharedPtr message)
{
    RCLCPP_INFO(node->get_logger(), "Received pointcloud with %s path.",
                message->floor_path.c_str());
}

void TrajectoryAvailable(const interface_msgs::msg::Trajectories::SharedPtr message)
{
    RCLCPP_INFO(node->get_logger(), "Received trajectory with %ld poses.",
                message->trajectories.back().poses.size());
}

void StateChanged(const std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Request> request,
                  std::shared_ptr<slam_manager_msgs::srv::SetSlamMode::Response>      response)
{
    RCLCPP_INFO(node->get_logger(), "StateChanged %d", request->mode.mode);
    response->success = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("stub_node");

    const auto export_map_service =
        node->create_service<interface_msgs::srv::SetString>("export_map", &ExportMap);
    const auto clear_map_service =
        node->create_service<std_srvs::srv::Trigger>("clear_map", &ClearMap);

    const auto modify_camera_service =
        node->create_service<interface_msgs::srv::CameraMovement>("modify_camera", &ModifyCamera);
    const auto move_camera_to_robot_service =
        node->create_service<std_srvs::srv::Trigger>("move_camera_to_robot", &MoveCameraToRobot);
    const auto follow_robot_service =
        node->create_service<interface_msgs::srv::FollowRobot>("follow_robot", &FollowRobot);
    const auto set_robot_location_to_camera_service = node->create_service<std_srvs::srv::Trigger>(
        "set_robot_location_to_camera", &SetRobotLocationToCamera);

    const auto get_software_versions_service = node->create_service<interface_msgs::srv::Listing>(
        "get_software_versions", &GetSoftwareVersions);

    const auto add_floor_service =
        node->create_service<interface_msgs::srv::SetString>("add_floor", &AddFloor);
    const auto delete_floor_service =
        node->create_service<interface_msgs::srv::ActionOnIndex>("delete_floor", &DeleteFloor);
    const auto clear_floor_service =
        node->create_service<std_srvs::srv::Trigger>("clear_floor", &ClearFloor);
    const auto get_floors_service =
        node->create_service<interface_msgs::srv::Listing>("get_floors", &GetFloors);
    const auto get_mapping_floor_service =
        node->create_service<interface_msgs::srv::GetIndex>("get_mapping_floor", &GetMappingFloor);
    const auto get_view_floor_service =
        node->create_service<interface_msgs::srv::GetIndex>("get_view_floor", &GetViewFloor);
    const auto select_view_floor_service = node->create_service<interface_msgs::srv::ActionOnIndex>(
        "select_view_floor", &SelectViewFloor);
    const auto select_mapping_floor_service =
        node->create_service<interface_msgs::srv::ActionOnIndex>("select_mapping_floor",
                                                                 &SelectMappingFloor);
    const auto set_floor_name_service =
        node->create_service<interface_msgs::srv::UpdateStringAt>("set_floor_name", &SetFloorName);

    const auto add_marker_service =
        node->create_service<interface_msgs::srv::SetString>("add_marker", &AddMarker);
    const auto remove_marker_service =
        node->create_service<interface_msgs::srv::ActionOnOptionalIndex>("remove_marker",
                                                                         &RemoveMarker);
    const auto get_markers_service =
        node->create_service<interface_msgs::srv::Listing>("get_markers", &GetMarkers);
    const auto set_marker_label_service =
        node->create_service<interface_msgs::srv::SetString>("set_marker_label", &SetMarkerLabel);
    const auto set_marker_label_at_service =
        node->create_service<interface_msgs::srv::UpdateStringAt>("set_marker_label_at",
                                                                  &SetMarkerLabelAt);

    const auto set_scale_unit_meters_service =
        node->create_service<std_srvs::srv::SetBool>("scale_unit_meters", &SetScaleUnitMeters);
    const auto set_video_resolution_mode_service =
        node->create_service<interface_msgs::srv::SetVideoResolutionMode>(
            "set_video_resolution_mode", &SetVideoResolutionMode);
    const auto set_video_stream_address_service =
        node->create_service<interface_msgs::srv::SetVideoStreamAddress>("set_video_stream_address",
                                                                         &SetVideoStreamAddress);
    const auto set_video_stream_service =
        node->create_service<std_srvs::srv::SetBool>("set_video_stream", &SetVideoStream);
    const auto clear_path_service =
        node->create_service<std_srvs::srv::Trigger>("clear_path", &ClearPath);

    const auto subscription_ = node->create_subscription<interface_msgs::msg::RobotStatus>(
        "robot_status", 10, RobotStatusChanged);

    const auto pointcloud_subscription =
        node->create_subscription<interface_msgs::msg::PointCloudAvailable>(
            "point_cloud_available", 10, PointCloudAvailable);
    const auto trajectory_subscription =
        node->create_subscription<interface_msgs::msg::Trajectories>("trajectories", 10,
                                                                     TrajectoryAvailable);
    const auto slam_subscription = node->create_service<slam_manager_msgs::srv::SetSlamMode>(
        "/slam/set_slam_mode", &StateChanged);


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
