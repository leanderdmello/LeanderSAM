// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// Wrapper class for stb image library
class StbImage
{
    public:

    void FlipVertical(std::vector<unsigned char> *data, int width, int height, int channels);
    int Load(const rclcpp::Logger &logger, const std::string &path, std::vector<unsigned char> *data, int *width, int *height, int *channels);
    void Save(const std::string &path, std::vector<unsigned char> *data, int width, int height, int channels);
};