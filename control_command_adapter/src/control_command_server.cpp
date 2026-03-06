// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "control_command_server.hpp"

// C++ Standard Library
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

ControlCommandServer::ControlCommandServer(rclcpp::Node::SharedPtr node, int port)
    : node_(node), port_(port)
{
}

void ControlCommandServer::Run()
{
    server_.init_asio();
    server_.set_message_handler(
        [this](websocketpp::connection_hdl                                 hdl,
               websocketpp::server<websocketpp::config::asio>::message_ptr msg)
        { this->on_websocket_message(hdl, msg); });
    server_.set_open_handler(
        [&](websocketpp::connection_hdl hdl)
        {
            connections_.push_back(hdl);
            RCLCPP_INFO(node_->get_logger(), "WebSocket client connected.");
        });
    server_.set_close_handler(
        [&](websocketpp::connection_hdl hdl)
        {
            auto index = std::find_if(connections_.cbegin(), connections_.cend(),
                                      [&hdl](const websocketpp::connection_hdl &a)
                                      {
                                          if(a.expired())
                                          {
                                              return false;
                                          }
                                          return a.lock().get() == hdl.lock().get();
                                      });
            if(index != connections_.end())
            {
                connections_.erase(index);
            }
            RCLCPP_INFO(node_->get_logger(), "WebSocket client disconnected.");
        });

    try
    {
        RCLCPP_INFO(node_->get_logger(), "ControlCommandServer: listening on port %d.", port_);
        server_.set_reuse_addr(true);
        server_.clear_access_channels(websocketpp::log::alevel::frame_header |
                                      websocketpp::log::alevel::frame_payload);
        server_.listen(port_);
        server_.start_accept();
        server_.run();
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "WebSocket server error: %s.", e.what());
    }
}

void ControlCommandServer::Stop()
{
    run_ = false;
    server_.stop_listening();
    for(auto &connection : connections_)
    {
        try
        {
            server_.close(connection, websocketpp::close::status::normal, "Server shutdown");
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error closing websocket connection: %s.", e.what());
        }
    }
    cv_.notify_one();
}

bool ControlCommandServer::Running()
{
    return run_;
}

std::vector<Request> ControlCommandServer::GetCommands()
{
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [&]() { return (!request_queue_.empty() || !run_); });
    if(request_queue_.empty())
    {
        return {};
    }
    else
    {
        auto to_process = request_queue_;
        request_queue_.clear();
        return to_process;
    }
}

void ControlCommandServer::on_websocket_message(
    websocketpp::connection_hdl                                 hdl,
    websocketpp::server<websocketpp::config::asio>::message_ptr msg)
{
    try
    {
        std::string                 payload = msg->get_payload();
        std::lock_guard<std::mutex> lock(mutex_);
        request_queue_.push_back({hdl, payload});
        cv_.notify_one();
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error processing WebSocket message: %s.", e.what());
    }
}

void ControlCommandServer::SendResponse(Response response)
{
    server_.send(response.Handle, response.Message, websocketpp::frame::opcode::text);
}
