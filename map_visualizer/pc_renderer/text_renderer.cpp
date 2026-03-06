// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "text_renderer.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "shader.h"

TextRenderer::TextRenderer(const rclcpp::Logger &logger)
    : logger_(logger), ft_lib_(std::make_unique<FT_Library>())
{
    offset_ = {0, 0};

    // Initialize freetype library
    if(FT_Init_FreeType(&*ft_lib_))
    {
        std::string e("ERROR::FREETYPE: Could not init FreeType Library");
        RCLCPP_ERROR(logger_, "%s", e.c_str());
        throw std::runtime_error(e);
    }

    // Only have a single font so no need to fuss about anything more complex
    std::string fontPath = ament_index_cpp::get_package_share_directory("map_visualizer") +
                           "/resources/MartianMono-Bold.ttf";
    LoadFont(fontPath);

    GLuint vertex_shader_   = compileShader(logger_, GL_VERTEX_SHADER, vert_src_);
    GLuint fragment_shader_ = compileShader(logger_, GL_FRAGMENT_SHADER, frag_src_);
    shader_                 = linkProgramFromShaders(logger_, {vertex_shader_, fragment_shader_});

    ConfigureObjects();
}

TextRenderer::~TextRenderer()
{
    // destroy FreeType once we're finished
    FT_Done_FreeType(*ft_lib_);

    for(auto &character : glyph_texture_)
        glDeleteTextures(1, &character);

    glBindVertexArray(VAO_);
    glDeleteVertexArrays(1, &VAO_);
    glDeleteBuffers(1, &VBO_);
    glBindVertexArray(0);
}

void TextRenderer::ConfigureObjects()
{
    // configure VAO/VBO for texture quads
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glBindVertexArray(VAO_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void TextRenderer::LoadFont(std::string fontPath)
{
    // Load font as face
    FT_Face face;

    if(FT_New_Face(*ft_lib_, fontPath.c_str(), 0, &face))
    {
        std::string e("ERROR::FREETYPE: Failed to load font");
        RCLCPP_ERROR(logger_, "%s", e.c_str());
        throw std::runtime_error(e);
    }
    else
    {
        // set size to load glyphs as
        FT_Set_Pixel_Sizes(face, 0, static_cast<unsigned int>(default_font_size_));

        // disable byte-alignment restriction
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        // load first 128 characters of ASCII set
        glyph_texture_.resize(128);
        for(unsigned char c = 0; c < glyph_texture_.size(); c++)
        {
            // Load character glyph
            if(FT_Load_Char(face, c, FT_LOAD_RENDER))
            {
                RCLCPP_ERROR(logger_, "ERROR::FREETYTPE: Failed to load Glyph");
                continue;
            }

            // generate texture
            glGenTextures(1, &glyph_texture_[c]);
            glBindTexture(GL_TEXTURE_2D, glyph_texture_[c]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, face->glyph->bitmap.width,
                         face->glyph->bitmap.rows, 0, GL_RED, GL_UNSIGNED_BYTE,
                         face->glyph->bitmap.buffer);

            // set texture options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // now store character for later use
            Character character = {glyph_texture_[c],
                                   glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
                                   glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
                                   static_cast<unsigned int>(face->glyph->advance.x)};
            characters_.insert(std::pair<char, Character>(c, character));
        }
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // destroy FreeType once we're finished
    FT_Done_Face(face);
}

void TextRenderer::Render(const glm::mat4 &projection, const glm::mat4 &view, Message message)
{
    // Activate shader. (when having lots of text elements to render, this can be shared)
    glUseProgram(shader_);

    // activate corresponding render state
    glUniform3f(glGetUniformLocation(shader_, "textColor"), message.Color.x, message.Color.y,
                message.Color.z);
    glUniformMatrix4fv(glGetUniformLocation(shader_, "projection"), 1, GL_FALSE,
                       glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shader_, "view"), 1, GL_FALSE, glm::value_ptr(view));

    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO_);

    auto scale = message.FontSize / default_font_size_;

    // TODO() We probably don't have to do this each render call, only when marker moves / things
    // dynamically change.
    glm::vec2 center_message_correction = {0.f, 0.f};
    if(message.Centered)
    {
        center_message_correction = GetCenterOffset(message);
    }

    // iterate through all characters
    std::string::const_iterator c;
    float message_position_x = message.position.x + center_message_correction.x;
    float message_position_y = message.position.y + center_message_correction.y;

    for(c = message.Text.begin(); c != message.Text.end(); c++)
    {
        Character &character = characters_[*c];

        float xpos = offset_.x + message_position_x + character.Bearing.x * scale;

        float ypos;
        if(message.Flipped)
            ypos = offset_.y + message_position_y - (character.Bearing.y) * scale;
        else
            ypos =
                offset_.y + message_position_y - (character.Size.y - character.Bearing.y) * scale;

        float w = character.Size.x * scale;
        float h = character.Size.y * scale;

        float verts[6][4];

        if(message.Flipped)
        {
            // flipped UV's on y axis -> 1 -
            verts[0][0] = xpos;
            verts[0][1] = ypos + h;
            verts[0][2] = 0.0f;
            verts[0][3] = 1.0f;
            verts[1][0] = xpos;
            verts[1][1] = ypos;
            verts[1][2] = 0.0f;
            verts[1][3] = 0.0f;
            verts[2][0] = xpos + w;
            verts[2][1] = ypos;
            verts[2][2] = 1.0f;
            verts[2][3] = 0.0f;

            verts[3][0] = xpos;
            verts[3][1] = ypos + h;
            verts[3][2] = 0.0f;
            verts[3][3] = 1.0f;
            verts[4][0] = xpos + w;
            verts[4][1] = ypos;
            verts[4][2] = 1.0f;
            verts[4][3] = 0.0f;
            verts[5][0] = xpos + w;
            verts[5][1] = ypos + h;
            verts[5][2] = 1.0f;
            verts[5][3] = 1.0f;
        }
        else
        {
            verts[0][0] = xpos;
            verts[0][1] = ypos + h;
            verts[0][2] = 0.0f;
            verts[0][3] = 0.0f;
            verts[1][0] = xpos;
            verts[1][1] = ypos;
            verts[1][2] = 0.0f;
            verts[1][3] = 1.0f;
            verts[2][0] = xpos + w;
            verts[2][1] = ypos;
            verts[2][2] = 1.0f;
            verts[2][3] = 1.0f;

            verts[3][0] = xpos;
            verts[3][1] = ypos + h;
            verts[3][2] = 0.0f;
            verts[3][3] = 0.0f;
            verts[4][0] = xpos + w;
            verts[4][1] = ypos;
            verts[4][2] = 1.0f;
            verts[4][3] = 1.0f;
            verts[5][0] = xpos + w;
            verts[5][1] = ypos + h;
            verts[5][2] = 1.0f;
            verts[5][3] = 0.0f;
        }

        // render glyph texture over quad
        glBindTexture(GL_TEXTURE_2D, character.TextureID);

        // update content of VBO memory
        glBindBuffer(GL_ARRAY_BUFFER, VBO_);

        // be sure to use glBufferSubData and not glBufferData
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // render quad
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // now advance cursors for next glyph (note that advance is number of 1/64 pixels)
        message_position_x += (character.Advance >> 6) *
                              scale; // bitshift by 6 to get value in pixels (2^6 = 64 (divide
                                     // amount of 1/64th pixels by 64 to get amount of pixels))
    }

    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

glm::vec2 TextRenderer::GetCenterOffset(Message &message)
{
    glm::vec2 offset = {0.f, 0.f};
    auto      scale  = message.FontSize / default_font_size_;

    std::string::const_iterator c;
    for(c = message.Text.begin(); c != message.Text.end(); c++)
    {
        Character &character = characters_[*c];

        float xpos = offset_.x + offset.x + character.Bearing.x * scale;
        offset.x += (character.Advance >> 6) * scale;

        float ypos = character.Size.y * scale;
        if(ypos > offset.y)
        {
            offset.y = ypos;
        }
    }

    // offset to the left
    return offset * -0.5f;
}

// Function to convert a hex color string (#RRGGBBAA or RRGGBB) to normalized RGBA float values
glm::vec3 TextRenderer::HexToGLColor(std::string value)
{
    std::string cleanedHex = (value[0] == '#') ? value.substr(1) : value;

    uint32_t          hexValue;
    std::stringstream ss;
    ss << std::hex << cleanedHex;
    ss >> hexValue;

    glm::vec3 rgb;
    if(cleanedHex.length() == 6)
    {
        rgb.x = ((hexValue >> 16) & 0xFF) / 255.0f;
        rgb.y = ((hexValue >> 8) & 0xFF) / 255.0f;
        rgb.z = (hexValue & 0xFF) / 255.0f;
    }
    else
    {
        RCLCPP_ERROR(logger_, "Invalid hex color format!");
        rgb.x = rgb.y = rgb.z = 1.0f; // Default to white
    }

    return rgb;
}