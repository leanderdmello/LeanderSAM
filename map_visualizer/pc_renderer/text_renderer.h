// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <map>
#include <memory>
#include <string>
#include <vector>

class TextRenderer
{
public:

    // Holds all information relavant to the message that will be displayed on screen
    struct Message {
        std::string Text;           // Storage for the message
        glm::vec2 position;         // X and Y position on screen where the message is displayed relative to origin
        float FontSize;             // Size of the font
        glm::vec3 Color;            // Color of the message to be displayed
        bool Centered = {false};    // Allight message around position
        bool Flipped = {true};      // EGL and GLFW inverse Y-axis, causing issues with rendering
    };

    /// Holds all state information relevant to a character as loaded using FreeType
    struct Character {
        unsigned int TextureID; // ID handle of the glyph texture
        glm::ivec2 Size;        // Size of glyph
        glm::ivec2 Bearing;     // Offset from baseline to left/top of glyph
        unsigned int Advance;   // Horizontal offset to advance to next glyph
    };

    TextRenderer(const rclcpp::Logger &logger);
    ~TextRenderer();
	
    void Render(const glm::mat4& projection, const glm::mat4& view, Message message);

    glm::vec3 HexToGLColor(std::string value);

private:

    void ConfigureObjects();
    void LoadFont(std::string fontPath);
    glm::vec2 GetCenterOffset(Message& message);

    // FreeType library
    std::unique_ptr<FT_Library> ft_lib_;

    std::map<GLchar, Character> characters_;
    GLuint shader_;
    unsigned int VAO_, VBO_;

    glm::i32vec2 offset_;

    float default_font_size_ = { 48.f };

    std::vector<GLuint> glyph_texture_;
    rclcpp::Logger logger_;

    const char* frag_src_ = R"(#version 430 core

    in vec2 TexCoords;
    out vec4 color;

    uniform sampler2D text;
    uniform vec3 textColor;

    void main() {    
        vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
        color = vec4(textColor, 1.0) * sampled;
    })";

    const char* vert_src_ = R"(#version 430 core

    layout (location = 0) in vec4 vertex;
    out vec2 TexCoords;

    uniform mat4 projection;
    uniform mat4 view;

    void main() {
        gl_Position = projection * view * vec4(vertex.xy, 0.0, 1.0);
        TexCoords = vertex.zw;
    })";
};