// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <filesystem>
#include "slam_manager_msgs/msg/slam_mode.hpp"

class FileAlreadyExistsException : public std::exception
{
public:
    FileAlreadyExistsException(const std::string &file) : message_(file + " already exists") {}

    const char *what() const noexcept override { return message_.c_str(); }

private:
    std::string message_{};
};

class FailedToChangeSlamStateException : public std::exception
{
public:
    FailedToChangeSlamStateException(uint8_t mode)
    {
        std::string mode_text{};
        switch(mode)
        {
        case slam_manager_msgs::msg::SlamMode::IDLE:
            mode_text = "Idle";
            break;
        case slam_manager_msgs::msg::SlamMode::MAPPING:
            mode_text = "Mapping";
            break;
        case slam_manager_msgs::msg::SlamMode::LOCALIZATION:
            mode_text = "Localization";
            break;
        default:
            mode_text = "Unknown";
            break;
        }
        message_ = "Failed to change mode to " + mode_text;
    }

    const char *what() const noexcept override { return message_.c_str(); }

private:
    std::string message_{};
};

class IndexOutOfRange : public std::exception
{
public:
    IndexOutOfRange(const std::string &method, const std::string &name, int index)
        : message_("Failed to " + method + " " + name + " at index " + std::to_string(index) +
                   ": Index out of range.")
    {
    }
    IndexOutOfRange(const std::string &method, int index)
        : message_("Failed to " + method + " at index " + std::to_string(index) +
                   ": Index out of range.")
    {
    }
    const char *what() const noexcept override { return message_.c_str(); }

private:
    std::string message_{};
};


class FloorNotSelected : public std::exception
{
public:
    FloorNotSelected(const std::string &method, const std::string &type)
        : message_("Failed to " + method + ": No " + type + " floor selected.")
    {
    }
    const char *what() const noexcept override { return message_.c_str(); }

private:
    std::string message_{};
};
