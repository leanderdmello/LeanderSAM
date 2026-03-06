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

class Robot : public HUDElement {
public:
    Robot(const rclcpp::Logger &logger, std::shared_ptr<Texture> texture, glm::vec2 position);
    ~Robot();

    void Render(const glm::mat4& view, const glm::mat4& projection, const glm::mat4& model);
    void UpdatePosition(const glm::vec2 position);
    void ResetPosition();
    glm::vec2 GetPosition() const;

private:

    void Place();

    std::optional<glm::vec2> position_;

    GLuint texture_;
    GLuint shader_;

    const char* frag_src_ = R"(#version 430 core

    in vec2 TexCoord;
    out vec4 FragColor;
    uniform sampler2D hudTexture;

    void main() {
        FragColor = texture(hudTexture, TexCoord);
    })";

    const char* vert_src_ = R"(#version 430 core

    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec2 aTexCoord;
    uniform mat4 projection;
    uniform mat4 view;
    uniform mat4 model;
    out vec2 TexCoord;

    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0, 1.0);
        TexCoord = aTexCoord;
    })";
};
