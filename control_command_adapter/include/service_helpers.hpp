// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once
#include <nlohmann/json.hpp>
#include "interface_msgs/srv/get_index.hpp"

#include "rclcpp/rclcpp.hpp"
#include "exceptions.hpp"
#include <chrono>

template <typename ServiceType>
nlohmann::json call_service_empty_response(typename rclcpp::Client<ServiceType>::SharedPtr &client,
                                           typename ServiceType::Request::SharedPtr         request,
                                           std::chrono::duration<int, std::milli> cmd_timeout)
{
    client->wait_for_service();
    auto result_future = client->async_send_request(request);
    if(result_future.wait_for(cmd_timeout) != std::future_status::ready)
    {
        throw ErrorProcessingRequestException("Failed to call service");
    }
    typename ServiceType::Response::SharedPtr response = result_future.get();
    if(!response->success)
    {
        throw ErrorProcessingRequestException(response->message);
    }
    else
    {
        return nullptr;
    }
    return nullptr;
}

template <typename ServiceType>
void call_service_no_wait(typename rclcpp::Client<ServiceType>::SharedPtr &client,
                          typename ServiceType::Request::SharedPtr         request)
{
    client->wait_for_service();
    auto result_future = client->async_send_request(request);
    (void)result_future;
}

template <typename ServiceType>
nlohmann::json call_service_with_list_response(
    typename rclcpp::Client<ServiceType>::SharedPtr &client,
    typename ServiceType::Request::SharedPtr         request,
    std::chrono::duration<int, std::milli>           cmd_timeout)
{
    client->wait_for_service();
    auto result_future = client->async_send_request(request);
    if(result_future.wait_for(cmd_timeout) != std::future_status::ready)
    {
        throw ErrorProcessingRequestException("Failed to call service");
    }
    typename ServiceType::Response::SharedPtr response = result_future.get();
    return nlohmann::json(response->data);
}

template <typename ServiceType>
nlohmann::json call_service_with_int_response(
    typename rclcpp::Client<ServiceType>::SharedPtr &client,
    typename ServiceType::Request::SharedPtr         request,
    std::chrono::duration<int, std::milli>           cmd_timeout)
{
    client->wait_for_service();
    auto result_future = client->async_send_request(request);
    if(result_future.wait_for(cmd_timeout) != std::future_status::ready)
    {
        throw ErrorProcessingRequestException("Failed to call service");
    }
    typename ServiceType::Response::SharedPtr response = result_future.get();
    if(response->index == interface_msgs::srv::GetIndex::Response::NONE)
    {
        throw NotFoundException();
    }
    return nlohmann::json(response->index);
}
