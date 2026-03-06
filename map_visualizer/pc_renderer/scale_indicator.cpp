// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "scale_indicator.h"
#include "shader.h"

#include <glm/gtc/type_ptr.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>

ScaleIndicator::ScaleIndicator(const rclcpp::Logger         &logger,
                               std::shared_ptr<TextRenderer> text_renderer,
                               float                         zoom,
                               float                         frame_width,
                               float                         real_world_distance)
    : text_renderer_(text_renderer),
      real_world_distance_(real_world_distance),
      zoom_(zoom),
      aspect_ratio_(1.0f), // Will be updated on first render
      frame_width_(frame_width),
      logger_(logger)
{
    // Initialize shader for rendering the scale bar
    const char *vert_src = R"(#version 430 core
    layout (location = 0) in vec2 position;

    uniform mat4 projection;
    uniform mat4 view;

    void main()
    {
        gl_Position = projection * view * vec4(position, 0.0, 1.0);
    }
    )";

    const char *frag_src = R"(#version 430 core
    uniform vec3 color;
    out vec4 FragColor;

    void main()
    {
        FragColor = vec4(color, 1.0);
    }
    )";

    auto vert_shader = compileShader(logger_, GL_VERTEX_SHADER, vert_src);
    auto frag_shader = compileShader(logger_, GL_FRAGMENT_SHADER, frag_src);
    bar_shader_      = linkProgramFromShaders(logger_, {vert_shader, frag_shader});

    // Initialize bar geometry
    InitializeBarGeometry();

    // Update bar geometry with initial zoom (aspect ratio will be updated on first render)
    UpdateBarGeometry(zoom, 1.0f, frame_width);

    // Initialize text message
    text_message_.Text     = "";
    text_message_.position = screen_position_;
    text_message_.FontSize = font_size_;
    text_message_.Color    = text_color_;
    text_message_.Centered = false;
    text_message_.Flipped  = true;
}

ScaleIndicator::~ScaleIndicator()
{
    if(bar_vao_ != 0)
    {
        glDeleteVertexArrays(1, &bar_vao_);
    }
    if(bar_vbo_ != 0)
    {
        glDeleteBuffers(1, &bar_vbo_);
    }
    if(bar_shader_ != 0)
    {
        glDeleteProgram(bar_shader_);
    }
}

void ScaleIndicator::InitializeBarGeometry()
{
    glGenVertexArrays(1, &bar_vao_);
    glGenBuffers(1, &bar_vbo_);

    glBindVertexArray(bar_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, bar_vbo_);

    // Initial buffer allocation (will be updated in UpdateBarGeometry)
    glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void ScaleIndicator::UpdateBarGeometry(float zoom, float aspect_ratio, float frame_width)
{
    // Update stored frame width to reflect current render surface
    frame_width_ = frame_width;

    // Dynamically adjust real_world_distance_ to keep bar width within reasonable bounds
    // This ensures the bar is always visible and readable regardless of zoom level
    real_world_distance_ = GetNiceDistanceForZoom(zoom, aspect_ratio, frame_width);

    // Calculate the on-screen width of the bar based on zoom level
    // The screen projection spans from -aspect_ratio to aspect_ratio (total width = 2 *
    // aspect_ratio) The map projection shows frame_width / zoom meters in world space Therefore: 1
    // meter in world space = (2 * aspect_ratio * zoom) / frame_width NDC units
    bar_width_ = (real_world_distance_ * zoom * 2.0f * aspect_ratio) / frame_width;

    // Create bar geometry as a filled rectangle (quad made of 2 triangles)
    // Bar grows to the LEFT from the position point (for right-side placement)
    float vertices[] = {
        // First triangle
        screen_position_.x - bar_width_,
        screen_position_.y,
        screen_position_.x,
        screen_position_.y,
        screen_position_.x - bar_width_,
        screen_position_.y + bar_height_,
        // Second triangle
        screen_position_.x,
        screen_position_.y,
        screen_position_.x,
        screen_position_.y + bar_height_,
        screen_position_.x - bar_width_,
        screen_position_.y + bar_height_,
    };

    glBindBuffer(GL_ARRAY_BUFFER, bar_vbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Update text to show the distance
    std::ostringstream oss;
    float              display_distance =
        use_meters_ ? real_world_distance_ : real_world_distance_ * meters_to_feet_;
    std::string unit_text = use_meters_ ? "m" : "ft";
    oss << std::fixed << std::setprecision(1) << display_distance << unit_text;
    text_message_.Text = oss.str();

    // Position text below the bar, aligned with the left edge of the bar
    text_message_.position = glm::vec2(screen_position_.x - bar_width_, screen_position_.y - 0.1f);
}

void ScaleIndicator::Render(const glm::mat4 &view,
                            const glm::mat4 &projection,
                            float            zoom,
                            float            aspect_ratio,
                            float            frame_width)
{
    // Update zoom, aspect_ratio, or frame_width if they have changed
    if(zoom != zoom_ || aspect_ratio != aspect_ratio_ || frame_width != frame_width_)
    {
        zoom_         = zoom;
        aspect_ratio_ = aspect_ratio;
        UpdateBarGeometry(zoom_, aspect_ratio_, frame_width);
    }

    // Render the scale bar
    glUseProgram(bar_shader_);
    glUniformMatrix4fv(glGetUniformLocation(bar_shader_, "view"), 1, GL_FALSE,
                       glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(bar_shader_, "projection"), 1, GL_FALSE,
                       glm::value_ptr(projection));
    glUniform3fv(glGetUniformLocation(bar_shader_, "color"), 1, glm::value_ptr(bar_color_));

    glBindVertexArray(bar_vao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    glUseProgram(0);

    // Render the text
    if(text_renderer_)
    {
        text_renderer_->Render(projection, view, text_message_);
    }
}

void ScaleIndicator::SetZoom(float zoom)
{
    if(zoom != zoom_)
    {
        zoom_ = zoom;
        UpdateBarGeometry(zoom_, aspect_ratio_, frame_width_);
    }
}

void ScaleIndicator::SetRealWorldDistance(float distance)
{
    if(distance > 0.0f && distance != real_world_distance_)
    {
        real_world_distance_ = distance;
        UpdateBarGeometry(zoom_, aspect_ratio_, frame_width_);
    }
}

void ScaleIndicator::SetPosition(const glm::vec2 &position)
{
    screen_position_ = position;
    UpdateBarGeometry(zoom_, aspect_ratio_, frame_width_);
}

float ScaleIndicator::GetRealWorldDistance() const
{
    return real_world_distance_;
}

float ScaleIndicator::GetZoom() const
{
    return zoom_;
}

void ScaleIndicator::SetUseMeters(bool use_meters)
{
    if(use_meters != use_meters_)
    {
        use_meters_ = use_meters;
        UpdateBarGeometry(zoom_, aspect_ratio_, frame_width_);
    }
}

float ScaleIndicator::GetNiceDistanceForZoom(float zoom,
                                             float aspect_ratio,
                                             float frame_width) const
{
    // Target bar width range (in NDC space)
    const float min_bar_width = 0.2f;
    const float max_bar_width = 0.5f;
    const float target_width  = (min_bar_width + max_bar_width) / 2.0f;

    // Conversion factor for feet to meters
    const float feet_to_meters = 0.3048f;

    float best_distance = 1.0f;
    float best_error    = std::numeric_limits<float>::max();

    // Try different orders of magnitude
    // Generate nice values: 1×10^n and 5×10^n (e.g., 1, 5, 10, 50, 100, 500, ...)
    for(int exponent = 0; exponent <= 4; ++exponent)
    {
        float base = std::pow(10.0f, exponent);

        // Try both 1×10^n and 5×10^n
        for(float multiplier : {1.0f, 2.5f, 5.0f})
        {
            float distance;

            if(use_meters_)
            {
                distance = multiplier * base;
            }
            else
            {
                // For feet: generate nice foot values and convert to meters
                float distance_feet = multiplier * base;
                distance            = distance_feet * feet_to_meters;
            }

            float bar_width = (distance * zoom * 2.0f * aspect_ratio) / frame_width;
            float error     = std::abs(bar_width - target_width);

            // Prefer values within the valid range, but accept any reasonable value
            if(bar_width >= min_bar_width && bar_width <= max_bar_width)
            {
                if(error < best_error)
                {
                    best_distance = distance;
                    best_error    = error;
                }
            }
            // If no value is in range yet, pick the closest one
            else if(best_error > (max_bar_width * 0.5f))
            {
                if(error < best_error)
                {
                    best_distance = distance;
                    best_error    = error;
                }
            }
        }
    }

    return best_distance;
}
