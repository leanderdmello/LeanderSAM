// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "interface_msgs/srv/set_video_stream_address.hpp"

class NetworkManager : public rclcpp::Node
{
public:
    NetworkManager() : Node("network_manager")
    {
        this->declare_parameter<std::string>("target_file",
                                             "/var/opt/telerob/static_stream_target.txt");

        this->get_parameter("target_file", target_file_);

        set_address_service_ = this->create_service<interface_msgs::srv::SetVideoStreamAddress>(
            "set_video_stream_address", std::bind(&NetworkManager::handle_set_address, this,
                                                  std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Network Manager Node has been started.");
    }

private:
    std::string                                                            target_file_;
    rclcpp::Service<interface_msgs::srv::SetVideoStreamAddress>::SharedPtr set_address_service_;

    void handle_set_address(
        const std::shared_ptr<interface_msgs::srv::SetVideoStreamAddress::Request> request,
        std::shared_ptr<interface_msgs::srv::SetVideoStreamAddress::Response>      response)
    {
        std::ofstream target_file_stream{target_file_, std::ios::out | std::ios::trunc};
        if(!target_file_stream.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open target file: %s",
                         target_file_.c_str());
            response->success = false;
            return;
        }

        target_file_stream << request->ip_address << ":" << request->port << std::endl;
        target_file_stream.close();

        RCLCPP_INFO(this->get_logger(), "Set video stream address to: %s:%u",
                    request->ip_address.c_str(), request->port);
        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NetworkManager>());
    rclcpp::shutdown();
    return 0;
}
