// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

struct Request
{
    websocketpp::connection_hdl Handle{};
    std::string                 Message{};
};

struct Response
{
    websocketpp::connection_hdl Handle{};
    std::string                 Message{};
};

class ControlCommandServer
{
public:
    ControlCommandServer(rclcpp::Node::SharedPtr node, int port);
    std::vector<Request> GetCommands();
    void                 SendResponse(Response response);

    void Run();
    void Stop();
    bool Running();

private:
    void on_websocket_message(websocketpp::connection_hdl                                 hdl,
                              websocketpp::server<websocketpp::config::asio>::message_ptr msg);

    websocketpp::server<websocketpp::config::asio> server_;
    rclcpp::Node::SharedPtr                        node_;
    std::vector<Request>                           request_queue_{};
    std::vector<websocketpp::connection_hdl>       connections_{};
    std::mutex                                     mutex_;
    std::condition_variable                        cv_;
    std::atomic<bool>                              run_{true};
    int                                            port_;
};
