// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include "text_renderer.h"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

// Scale indicator element that displays a scale bar with distance text
// The bar represents a fixed real-world distance in meters, and its on-screen length
// varies based on the current zoom level
class ScaleIndicator
{
public:
    ScaleIndicator(const rclcpp::Logger         &logger,
                   std::shared_ptr<TextRenderer> text_renderer,
                   float                         zoom,
                   float                         frame_width,
                   float real_world_distance = 10.0f); // 10 meters by default
    ~ScaleIndicator();

    // Render both the scale bar and distance text
    // view and projection are the HUD view/projection matrices (screen space)
    // zoom is needed to calculate the on-screen bar length based on world coordinates
    // aspect_ratio is needed to convert world units to screen space properly
    // frame_width is the current render surface width in pixels
    void Render(const glm::mat4 &view,
                const glm::mat4 &projection,
                float            zoom,
                float            aspect_ratio,
                float            frame_width);

    // Update the zoom level (used to recalculate bar length)
    void SetZoom(float zoom);

    // Set the real-world distance that the bar represents (in meters)
    void SetRealWorldDistance(float distance);

    // Set the position of the scale indicator in screen space (NDC coordinates)
    void SetPosition(const glm::vec2 &position);

    // Set display unit (true = meters, false = feet)
    void SetUseMeters(bool use_meters);

    float GetRealWorldDistance() const;
    float GetZoom() const;

private:
    // Initialize VAO and VBO for the scale bar
    void InitializeBarGeometry();

    // Update the bar geometry based on current zoom level
    void UpdateBarGeometry(float zoom, float aspect_ratio, float frame_width);

    // Find the nearest "nice" distance value (1, 2, 5, 10, 20, 50, 100, etc.)
    // that results in a bar width within reasonable bounds
    float GetNiceDistanceForZoom(float zoom, float aspect_ratio, float frame_width) const;

    std::shared_ptr<TextRenderer> text_renderer_;
    TextRenderer::Message         text_message_;

    float real_world_distance_; // in meters
    float zoom_;
    float aspect_ratio_;      // for world-to-screen conversion
    float frame_width_;       // actual frame width in pixels
    bool  use_meters_ = true; // display unit mode

    const rclcpp::Logger logger_;
    const float          meters_to_feet_ = 3.28084f;

    // Position in screen space (NDC coordinates)
    glm::vec2 screen_position_{-0.95f, -0.75f}; // Bottom-left corner, below floor indicator

    // Rendering parameters
    const float     bar_height_{0.02f};            // Height of the bar in NDC space
    const glm::vec3 bar_color_{0.0f, 0.0f, 0.0f};  // black
    const float     font_size_{0.04f};             // Font size for distance text
    const glm::vec3 text_color_{0.0f, 0.0f, 0.0f}; // black

    // OpenGL resources for bar rendering
    GLuint bar_vao_    = 0;
    GLuint bar_vbo_    = 0;
    GLuint bar_shader_ = 0;

    // Current calculated bar width (in NDC space)
    float bar_width_ = 0.0f;
};
