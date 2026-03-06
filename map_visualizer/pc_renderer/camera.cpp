// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "camera.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <vector>

#include "shader.h"

Camera2D::Camera2D(const rclcpp::Logger    &logger,
                   std::shared_ptr<Texture> texture,
                   glm::i32vec2             viewport,
                   glm::vec2                position,
                   float                    zoom)
    : position_(position), zoom_(zoom), window_width_(viewport.x), window_height_(viewport.y)
{
    std::string texture_path =
        ament_index_cpp::get_package_share_directory("map_visualizer") + "/resources/crosshair.png";
    texture_ = texture->Load(logger, texture_path);

    GLuint vertex_shader_   = compileShader(logger, GL_VERTEX_SHADER, vert_src_);
    GLuint fragment_shader_ = compileShader(logger, GL_FRAGMENT_SHADER, frag_src_);
    shader_                 = linkProgramFromShaders(logger, {vertex_shader_, fragment_shader_});

    // place marker
    Place();
}

Camera2D::~Camera2D()
{
    glDeleteTextures(1, &texture_);
    glDeleteProgram(shader_);
}

void Camera2D::SetAbsZoom(float zoom)
{
    zoom_ = zoom;
}

glm::mat4 Camera2D::GetViewMatrix() const
{
    return glm::translate(
        glm::rotate(glm::mat4(1.f), glm::radians(-rotation_), glm::vec3(0.f, 0.f, 1.f)),
        glm::vec3(-position_, 0.f));
}

glm::mat4 Camera2D::GetMapProjectionMatrix() const
{
    float half_w = (window_width_ * 0.5f) / zoom_;
    float half_h = (window_height_ * 0.5f) / zoom_;
    return glm::ortho(-half_w, half_w, -half_h, half_h, 1.f, 1.f);
}

glm::mat4 Camera2D::GetHUDProjectionMatrix() const
{
    float half_w = (hud_factor_ * window_width_ * 0.5f) / zoom_;
    float half_h = (hud_factor_ * window_height_ * 0.5f) / zoom_;

    return glm::ortho(-half_w, half_w, -half_h, half_h);
}

void Camera2D::Move(const glm::vec2 &delta)
{
    position_ += delta;
    // Update quads
    Place();
}

void Camera2D::Zoom(float factor)
{
    zoom_ = glm::clamp(zoom_ * factor, min_zoom_, max_zoom_);
    // Update quads
    Place();
}

void Camera2D::Rotate(float angle_deg)
{
    rotation_ += angle_deg;
    // Update quads
    Place();
}

void Camera2D::Resize(glm::i32vec2 viewport)
{
    window_width_  = viewport.x;
    window_height_ = viewport.y;
    // Update camera icon size after viewport change
    Place();
}

float Camera2D::GetZoom() const
{
    return zoom_;
}

glm::vec2 Camera2D::GetPosition() const
{
    return position_;
}

float Camera2D::GetRotation() const
{
    return rotation_;
}

float Camera2D::GetRotationRadians() const
{
    return glm::radians(rotation_);
}

void Camera2D::SetPosition(const glm::vec2 position)
{
    position_ = position;
    // Update quads
    Place();
}

void Camera2D::SetAbsRotation(float angle_deg)
{
    rotation_ = angle_deg;
    // Update quads
    Place();
}

void Camera2D::Place()
{
    float width  = (window_width_ / 15.f) / zoom_;
    float height = (window_height_ / 15.f) / zoom_;

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

void Camera2D::Render(const glm::mat4 &view, const glm::mat4 &projection)
{
    glm::mat4 model = glm::mat4(1.0f);
    RenderElement(shader_, texture_, view, projection, model);
}
