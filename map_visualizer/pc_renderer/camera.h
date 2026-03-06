// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include "hud_element.h"
#include "texture.h"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>

// Simple 2D orthogonal top-down camera
class Camera2D : public HUDElement
{
public:
    Camera2D(const rclcpp::Logger    &logger,
             std::shared_ptr<Texture> texture,
             glm::i32vec2             viewport,
             glm::vec2                position,
             float                    zoom);
    ~Camera2D();

    glm::mat4 GetViewMatrix() const;
    glm::mat4 GetMapProjectionMatrix() const;
    glm::mat4 GetHUDProjectionMatrix() const;

    void      Move(const glm::vec2 &delta);
    void      Zoom(float factor);
    void      Rotate(float angle_deg);
    void      Resize(glm::i32vec2 viewport);
    void      Render(const glm::mat4 &view, const glm::mat4 &projection);
    void      SetPosition(const glm::vec2 position);
    float     GetZoom() const;
    void      SetAbsZoom(float zoom);
    glm::vec2 GetPosition() const;
    float     GetRotation() const;
    float     GetRotationRadians() const;
    void      SetAbsRotation(float angle_deg);

private:
    void Place();

    glm::vec2 position_;
    float     rotation_{0.f};
    float     zoom_;
    float     window_width_, window_height_;

    const float hud_factor_{1.f};

    const float min_zoom_{0.05f};
    const float max_zoom_{200.f};

    GLuint texture_;
    GLuint shader_;

    const char *frag_src_ = R"(#version 430 core

    in vec2 TexCoord;
    out vec4 FragColor;
    uniform sampler2D hudTexture;

    void main() {
        FragColor = texture(hudTexture, TexCoord);
    })";

    const char *vert_src_ = R"(#version 430 core

    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec2 aTexCoord;
    uniform mat4 projection;
    uniform mat4 view;
    uniform mat4 model;
    out vec2 TexCoord;

    void main() {
        gl_Position = projection * view * model * vec4(aPos, 0.0, 1.0);
        TexCoord = aTexCoord;
    })";
};
