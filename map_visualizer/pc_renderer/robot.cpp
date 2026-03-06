// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "robot.h"

#include <glm/gtc/matrix_transform.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <vector>

#include "shader.h"

Robot::Robot(const rclcpp::Logger &logger, std::shared_ptr<Texture> texture, glm::vec2 position)
    : position_(position)
{
    std::string texture_path =
        ament_index_cpp::get_package_share_directory("map_visualizer") + "/resources/robot.png";
    texture_ = texture->Load(logger, texture_path);

    GLuint vertex_shader_   = compileShader(logger, GL_VERTEX_SHADER, vert_src_);
    GLuint fragment_shader_ = compileShader(logger, GL_FRAGMENT_SHADER, frag_src_);
    shader_                 = linkProgramFromShaders(logger, {vertex_shader_, fragment_shader_});

    // place marker
    Place();
}

Robot::~Robot()
{
    glDeleteTextures(1, &texture_);
    glDeleteProgram(shader_);
}

glm::vec2 Robot::GetPosition() const
{
    if(position_.has_value())
        return position_.value();
    else
        return glm::vec2(0.f, 0.f);
}

void Robot::Place()
{
    if(position_.has_value())
    {
        auto position = position_.value();
        // robot can be a bit bigger
        float width  = 0.334f;
        float height = 0.421f;

        // update quad vertex positions
        _quad[0] = position.x - (width * 0.5f); // bottom left x, y
        _quad[1] = position.y - (height * 0.5f);

        _quad[4] = position.x + (width * 0.5f); // bottom right x, y
        _quad[5] = position.y - (height * 0.5f);

        _quad[8] = position.x - (width * 0.5f); // top left x, y
        _quad[9] = position.y + (height * 0.5f);

        _quad[12] = position.x + (width * 0.5f); // top right x, y
        _quad[13] = position.y + (height * 0.5f);

    }
    else
    {
        // place outside of view
        _quad[0]  = -100.f;
        _quad[1]  = -100.f;
        _quad[4]  = -100.f;
        _quad[5]  = -100.f;
        _quad[8]  = -100.f;
        _quad[9]  = -100.f;
        _quad[12] = -100.f;
        _quad[13] = -100.f;
    }
    UpdateResources();
}

void Robot::Render(const glm::mat4 &view, const glm::mat4 &projection, const glm::mat4 &model)
{
    RenderElement(shader_, texture_, view, projection, model);
}

void Robot::UpdatePosition(const glm::vec2 position)
{
    position_ = position;
    // Update quads
    Place();
}

void Robot::ResetPosition()
{
    position_ = std::nullopt;
    Place();
}
