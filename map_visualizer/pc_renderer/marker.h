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
#include <string>

#include "text_renderer.h"
#include <rclcpp/rclcpp.hpp>

class Marker : public HUDElement {
    public:

    // Important to create and destroy the instance within GL context!
    Marker(const rclcpp::Logger &logger, std::shared_ptr<Texture> texture, std::shared_ptr<TextRenderer> text_renderer,
        glm::vec2 position, const std::string label);
    ~Marker();

    void Render(const glm::mat4& view, const glm::mat4& projection,
        const glm::mat4& model, bool highlight);
    void UpdateLabel(const std::string label);
    glm::vec2 GetPosition();

    private:

    void Place();

    std::shared_ptr<TextRenderer> text_renderer_;

    glm::vec2 position_;
    
    TextRenderer::Message message_;

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
        gl_Position = projection * model * view * vec4(aPos, 0.0, 1.0);
        TexCoord = aTexCoord;
    })";
};
