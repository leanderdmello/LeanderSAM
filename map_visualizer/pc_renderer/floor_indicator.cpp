// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "floor_indicator.h"

FloorIndicator::FloorIndicator(const rclcpp::Logger         &logger,
                               std::shared_ptr<TextRenderer> text_renderer,
                               const std::string            &floor_name)
    : text_renderer_(text_renderer), floor_name_(floor_name), logger_(logger)
{
    // Construct text message
    message_.Text     = floor_name_;
    message_.position = screen_position_;
    message_.FontSize = font_size_;
    message_.Color    = text_color_;
    message_.Centered = false; // Position directly at top-left
    message_.Flipped  = true;  // EGL and GLFW inverse Y-axis
}

FloorIndicator::~FloorIndicator() {}

void FloorIndicator::Render(const glm::mat4 &view, const glm::mat4 &projection)
{
    text_renderer_->Render(projection, view, message_);
}

void FloorIndicator::SetFloorName(const std::string &floor_name)
{
    floor_name_   = floor_name;
    message_.Text = floor_name_;
}

void FloorIndicator::SetPosition(const glm::vec2 &position)
{
    screen_position_  = position;
    message_.position = position;
}

std::string FloorIndicator::GetFloorName() const
{
    return floor_name_;
}
