// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "marker.h"
#include "shader.h"

Marker::Marker(const rclcpp::Logger &logger, std::shared_ptr<Texture> texture, std::shared_ptr<TextRenderer> text_renderer, 
    glm::vec2 position, const std::string label) :
    position_(position),
    text_renderer_(text_renderer)
{
    std::string texture_path = ament_index_cpp::get_package_share_directory("map_visualizer") + "/resources/marker.png";
    texture_ = texture->Load(logger, texture_path);

    GLuint vertex_shader_ = compileShader(logger, GL_VERTEX_SHADER, vert_src_);
    GLuint fragment_shader_ = compileShader(logger, GL_FRAGMENT_SHADER, frag_src_);
    shader_ = linkProgramFromShaders(logger, {vertex_shader_, fragment_shader_});

    // construct text message
    message_.Centered = true;
    message_.Color = text_renderer_->HexToGLColor("#12290e");
    message_.FontSize = 0.8;
    message_.position = position_ - glm::vec2(0.f, 0.2f); // above marker
    message_.Text = label;

    // place marker
    Place();
}

Marker::~Marker() {
    glDeleteTextures(1, &texture_);
    glDeleteProgram(shader_);
}

void Marker::Place() {

    float width = 0.75f;
    float height = 0.75f;

    // update quad vertex positions
    _quad[0] = position_.x - height; // bottom left x, y
    _quad[1] = position_.y - height;

    _quad[4] = position_.x + height; // bottom right x, y
    _quad[5] = position_.y - height;

    _quad[8] = position_.x - height; // top left x, y
    _quad[9] = position_.y + height;

    _quad[12] = position_.x + height; // top right x, y
    _quad[13] = position_.y + height;

    UpdateResources();
}

glm::vec2 Marker::GetPosition() {
    return position_;
}

void Marker::Render(const glm::mat4& view, const glm::mat4& projection,
    const glm::mat4& model, bool highlight) {
    RenderElement(shader_, texture_, view, projection, model);
    
    // First render highlight. NOT IMPLEMENTED
    if(highlight) {
        auto backdrop = message_;
        // inverse of current color. FFFFFF - current hex, but already converted
        backdrop.Color = glm::vec3(1.0f, 1.0f, 1.0f) - backdrop.Color;
    }

    text_renderer_->Render(projection, view, message_);
}

void Marker::UpdateLabel(const std::string label) {
    message_.Text = label;
}