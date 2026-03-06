// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <glad/glad.h>

#include "camera.h"
#include "floor_indicator.h"
#include "marker.h"
#include "point_cloud.h"
#include "robot.h"
#include "scale_indicator.h"
#include "texture.h"
#include "text_renderer.h"
#include <rclcpp/rclcpp.hpp>

class Map_Renderer
{
public:
    Map_Renderer(const rclcpp::Logger &logger, int frame_width, int frame_height);
    ~Map_Renderer();

    void AddPointCloudMap(const std::string uri);
    void AddPointCloudMap(void *data, size_t length);

    void AddPointCloudTrajectory(const std::string hex_color, const std::string uri);
    void AddPointCloudTrajectory(const std::string hex_color, void *data, size_t length);
    void ClearTrajectory();
    void ClearMap();

    void MoveCameraOnXAxis(float x);
    void MoveCameraOnYAxis(float y);
    void RotateCameraAroundZAxis(float angle_deg);
    void UpdateCameraZoom(float zoom);

    void GetCameraPosition(float &x, float &y);
    void SetCameraPosition(float x, float y);
    void SetAbsCameraPosition(float x, float y);
    void GetCameraPose(glm::vec2 &position, glm::vec4 &orientation);


    float GetAbsZoom();
    void  SetAbsZoom(float zoom);
    void  ResetZoom();
    float GetAbsRotation();
    void  SetAbsRotation(float angle_deg);

    void AddMarker(const int index, const std::string label, float x, float y);
    void RemoveMarker(const int index);
    void RemoveAllMarkers();
    void UpdateMarkerLabel(const int index, const std::string label);

    void UpdateRobotPosition(const float x, const float y);
    void UpdateRobotOrientation(const float z);
    void ResetRobotPosition();
    void MoveCameraToRobotPosition();
    void FollowRobotPosition();
    void FollowRobotOff();
    void FollowRobotOn(bool with_orientation);
    void GetFollowRobotStatus(bool &is_following, bool &is_following_orientation);

    void        SetFloorName(const std::string &floor_name);
    std::string GetFloorName() const;

    void SetScaleUnitMeters(bool use_meters);

    void      ResizeEGLSurface(int width, int height);
    glm::vec3 HexToGLColor(std::string value);

    void GetCurrentReslution(int &width, int &height);
    void GetExportResolution(int &width, int &height);
    void GetExportCenter(float &x, float &y);

    // Allow rendering of externally managed point clouds
    void SetPointCloudSources(PointCloud *map, PointCloud *trajectory);

    // TODO() make reference
    std::vector<uint8_t> Render(bool renderTrajectory = true,
                                bool renderRobot      = true,
                                bool renderCamera     = true);

    std::unique_ptr<PointCloud> map_;
    std::unique_ptr<PointCloud> trajectory_;

private:
    // use EGL for headerless rendering (no monitor)
    EGLDisplay eglDisplay_ = EGL_NO_DISPLAY;
    EGLContext eglContext_ = EGL_NO_CONTEXT;
    EGLSurface eglSurface_ = EGL_NO_SURFACE;
    EGLConfig  eglConfig_  = nullptr;

    std::unique_ptr<Camera2D> camera_;
    std::unique_ptr<Robot>    robot_;

    bool  follow_robot_       = {false};
    bool  follow_orientation_ = {false};
    float robot_orientation_  = {0.f}; // radians

    // Point cloud data

    // Rendering resolution
    int frame_width_;
    int frame_height_;

    // Default parameters
    const float POINT_SIZE_PX = 1.f;
    const float NEAR_DIST     = 0.05f;
    const float FAR_DIST      = 500.0f;

    const glm::vec2 default_camera_position_{0.f, 0.f};
    glm::vec2       follow_robot_offset_{0.f, 0.f};
    float           follow_robot_rotation_offset_{0.f};
    const float     default_camera_zoom_ = 5.f;

    // Frame to render to is a simple quad
    GLuint vao_, vbo_, ebo_, fbo_, rbo_;

    // Framebuffer texture
    GLuint color_texture_;

    // Quad program to project color texture to screen/framebuffer
    GLuint quadProg_;

    // quad shader sources
    GLuint vertex_shader_, fragment_shader_;

    // Pixel buffer
    std::vector<uint8_t> pixels_;

    // Optional external point cloud pointers (non-owning)
    PointCloud *external_map_{nullptr};
    PointCloud *external_trajectory_{nullptr};

    // Load textures for markers, robot and other hud elements
    std::shared_ptr<Texture> hud_texture_;

    // Text renderer
    std::shared_ptr<TextRenderer> text_renderer_;

    // Floor indicator
    std::unique_ptr<FloorIndicator> floor_indicator_;

    // Scale indicator
    std::unique_ptr<ScaleIndicator> scale_indicator_;

    // container of n markers
    std::map<int, std::shared_ptr<Marker>> markers_;
    rclcpp::Logger                         logger_;
};
