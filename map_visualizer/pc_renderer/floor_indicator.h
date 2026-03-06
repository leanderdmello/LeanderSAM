// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include "text_renderer.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// Floor indicator element that displays the current floor name in the top-left corner
class FloorIndicator
{
public:
    FloorIndicator(const rclcpp::Logger         &logger,
                   std::shared_ptr<TextRenderer> text_renderer,
                   const std::string            &floor_name);
    ~FloorIndicator();

    void        Render(const glm::mat4 &view, const glm::mat4 &projection);
    void        SetFloorName(const std::string &floor_name);
    void        SetPosition(const glm::vec2 &position);
    std::string GetFloorName() const;

private:
    std::shared_ptr<TextRenderer> text_renderer_;
    TextRenderer::Message         message_;
    std::string                   floor_name_;

    const rclcpp::Logger logger_;

    // Top-left corner position in screen space (NDC coordinates)
    glm::vec2       screen_position_{-0.95f, -0.9f};
    const float     font_size_{0.05f};             // Adjusted for NDC space (-1 to 1)
    const glm::vec3 text_color_{0.0f, 0.0f, 0.0f}; // black
};
