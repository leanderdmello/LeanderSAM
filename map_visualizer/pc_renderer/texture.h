// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include "stb_wrapper.h"

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <rclcpp/rclcpp.hpp>

class Texture {
    public:

    GLuint Load(const rclcpp::Logger &logger, std::string uri);

    private:

    StbImage stbImage_;
};
