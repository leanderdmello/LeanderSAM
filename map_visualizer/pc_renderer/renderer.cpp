// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "renderer.h"

#include <tracy/Tracy.hpp>
#include <tracy/TracyOpenGL.hpp>

#include <iostream>
#include <sstream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "quad_shaders.h"
#include "shader.h"

#pragma region openGL setup

Map_Renderer::Map_Renderer(const rclcpp::Logger &logger, int frame_width, int frame_height)
    : logger_(logger), frame_width_(frame_width), frame_height_(frame_height)
{
    // Initialize EGL
    eglDisplay_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if(eglDisplay_ == EGL_NO_DISPLAY)
    {
        RCLCPP_ERROR(logger_, "Failed to get EGL display");
        exit(-1);
    }

    if(!eglInitialize(eglDisplay_, nullptr, nullptr))
    {
        RCLCPP_ERROR(logger_, "Failed to initialize EGL");
        exit(-1);
    }

    // Choose a config supporting OpenGL and PBuffer
    const EGLint configAttribs[] = {EGL_SURFACE_TYPE,
                                    EGL_PBUFFER_BIT,
                                    EGL_RENDERABLE_TYPE,
                                    EGL_OPENGL_BIT,
                                    EGL_RED_SIZE,
                                    8,
                                    EGL_GREEN_SIZE,
                                    8,
                                    EGL_BLUE_SIZE,
                                    8,
                                    EGL_ALPHA_SIZE,
                                    8,
                                    EGL_DEPTH_SIZE,
                                    24,
                                    EGL_STENCIL_SIZE,
                                    8,
                                    EGL_NONE};

    EGLint numConfigs = 0;
    if(!eglChooseConfig(eglDisplay_, configAttribs, &eglConfig_, 1, &numConfigs) || numConfigs == 0)
    {
        RCLCPP_ERROR(logger_, "Failed to choose EGL config");
        eglTerminate(eglDisplay_);
        exit(-1);
    }

    // Bind the OpenGL API
    if(!eglBindAPI(EGL_OPENGL_API))
    {
        RCLCPP_ERROR(logger_, "Failed to bind OpenGL API to EGL");
        eglTerminate(eglDisplay_);
        exit(-1);
    }

    // Create a pbuffer surface
    const EGLint pbufferAttribs[] = {EGL_WIDTH, frame_width_, EGL_HEIGHT, frame_height_, EGL_NONE};
    eglSurface_ = eglCreatePbufferSurface(eglDisplay_, eglConfig_, pbufferAttribs);
    if(eglSurface_ == EGL_NO_SURFACE)
    {
        RCLCPP_ERROR(logger_, "Failed to create EGL pbuffer surface");
        eglTerminate(eglDisplay_);
        exit(-1);
    }

    // Create an OpenGL 4.3 core profile context if supported (requires EGL_KHR_create_context)
    const EGLint contextAttribs[] = {
        EGL_CONTEXT_MAJOR_VERSION_KHR, 4, EGL_CONTEXT_MINOR_VERSION_KHR, 3,
        // For core profile you may need EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR if available:
        // EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR,
        EGL_NONE};

    eglContext_ = eglCreateContext(eglDisplay_, eglConfig_, EGL_NO_CONTEXT, contextAttribs);

    if(eglContext_ == EGL_NO_CONTEXT)
    {
        // Fallback: try without version hints (let driver choose)
        eglContext_ = eglCreateContext(eglDisplay_, eglConfig_, EGL_NO_CONTEXT, nullptr);
        if(eglContext_ == EGL_NO_CONTEXT)
        {
            RCLCPP_ERROR(logger_, "Failed to create EGL context");
            eglDestroySurface(eglDisplay_, eglSurface_);
            eglTerminate(eglDisplay_);
            exit(-1);
        }
    }

    // Make context current
    if(!eglMakeCurrent(eglDisplay_, eglSurface_, eglSurface_, eglContext_))
    {
        RCLCPP_ERROR(logger_, "Failed to make EGL context current");
        eglDestroyContext(eglDisplay_, eglContext_);
        eglDestroySurface(eglDisplay_, eglSurface_);
        eglTerminate(eglDisplay_);
        exit(-1);
    }

    // Load GL functions via glad using eglGetProcAddress
    if(!gladLoadGLLoader((GLADloadproc)eglGetProcAddress))
    {
        RCLCPP_ERROR(logger_, "Failed to load GL via glad+eglGetProcAddress");
        exit(-1);
    }

    RCLCPP_INFO(logger_, "GL version: %s", glGetString(GL_VERSION));

    // Initialize Tracy GPU profiling context
    TracyGpuContext;

    // configure global opengl state
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    // Enable face culling
    glEnable(GL_CULL_FACE);
    // Cull back faces
    glCullFace(GL_BACK);
    // Define front faces as counter-clockwise
    glFrontFace(GL_CCW);

    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Frame buffer object
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // Color texture
    glGenTextures(1, &color_texture_);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, frame_width_, frame_height_, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture_, 0);

    // Depth buffer
    glGenRenderbuffers(1, &rbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, frame_width_, frame_height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo_);

    // Quad program to project color texture to screen
    vertex_shader_   = compileShader(logger_, GL_VERTEX_SHADER, quad_vert_src);
    fragment_shader_ = compileShader(logger_, GL_FRAGMENT_SHADER, quad_frag_src);
    quadProg_        = linkProgramFromShaders(logger_, {vertex_shader_, fragment_shader_});

    glUseProgram(quadProg_);
    glUniform1i(glGetUniformLocation(quadProg_, "u_tex"), 0);

    // Fullscreen quad setup (NDC coords)
    float quadVerts[] = {
        // pos (x,y)    uv (u,v)
        -1.0f, -1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f,  0.0f, 1.0f, 1.0f, 1.0f,  1.0f, 1.0f,
    };

    GLuint quadIdx[] = {0, 1, 2, 2, 1, 3};

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIdx), quadIdx, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
    glBindVertexArray(0);

    hud_texture_   = std::make_shared<Texture>();
    text_renderer_ = std::make_shared<TextRenderer>(logger_);

    // initial camera state
    camera_ =
        std::make_unique<Camera2D>(logger_, hud_texture_, glm::i32vec2(frame_width_, frame_height_),
                                   default_camera_position_, default_camera_zoom_);

    // Initialize floor indicator
    floor_indicator_ = std::make_unique<FloorIndicator>(logger_, text_renderer_, "");

    // Initialize scale indicator
    scale_indicator_ = std::make_unique<ScaleIndicator>(logger_, text_renderer_,
                                                        default_camera_zoom_, frame_width_, 10.0f);

    glDisable(GL_DEPTH_TEST);

    robot_ = std::make_unique<Robot>(logger_, hud_texture_, glm::vec2(0.f, 0.f));
    camera_->SetPosition(glm::vec2(0.f, 0.f));
}

Map_Renderer::~Map_Renderer()
{
    // cleanup
    map_        = nullptr;
    trajectory_ = nullptr;

    // delete all markers
    markers_.clear();

    robot_         = nullptr;
    text_renderer_ = nullptr;

    glDeleteTextures(1, &color_texture_);

    glDeleteProgram(quadProg_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    glDeleteVertexArrays(1, &vao_);

    glDeleteFramebuffers(1, &fbo_);
    glDeleteRenderbuffers(1, &rbo_);

    // Destroy EGL context/surface
    if(eglDisplay_ != EGL_NO_DISPLAY)
    {
        eglMakeCurrent(eglDisplay_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if(eglContext_ != EGL_NO_CONTEXT)
        {
            eglDestroyContext(eglDisplay_, eglContext_);
            eglContext_ = EGL_NO_CONTEXT;
        }
        if(eglSurface_ != EGL_NO_SURFACE)
        {
            eglDestroySurface(eglDisplay_, eglSurface_);
            eglSurface_ = EGL_NO_SURFACE;
        }
        eglTerminate(eglDisplay_);
        eglDisplay_ = EGL_NO_DISPLAY;
    }
}

float Map_Renderer::GetAbsZoom()
{
    return camera_->GetZoom();
}

void Map_Renderer::SetAbsZoom(float zoom)
{
    camera_->SetAbsZoom(zoom);
}

void Map_Renderer::ResetZoom()
{
    camera_->SetAbsZoom(default_camera_zoom_);
}

void Map_Renderer::SetPointCloudSources(PointCloud *map, PointCloud *trajectory)
{
    external_map_        = map;
    external_trajectory_ = trajectory;
}

#pragma region pointcloud
void Map_Renderer::AddPointCloudMap(void *data, size_t length)
{
    if(!map_)
        map_ = std::make_unique<PointCloud>(logger_, frame_width_, frame_height_, false);

    map_->InitMap(data, length);
}

void Map_Renderer::AddPointCloudMap(std::string uri)
{
    if(!map_)
        map_ = std::make_unique<PointCloud>(logger_, frame_width_, frame_height_, false);

    map_->InitMap(uri);
}

void Map_Renderer::AddPointCloudTrajectory(const std::string hex_color, const std::string uri)
{
    if(map_ == nullptr)
        return;

    if(!trajectory_)
        trajectory_ = std::make_unique<PointCloud>(logger_, frame_width_, frame_height_, true);

    trajectory_->UseSharedTextures(map_->GetColorTexture());

    glm::vec3 color = HexToGLColor(hex_color);
    trajectory_->InitTrajectory(color, uri);
}

void Map_Renderer::AddPointCloudTrajectory(const std::string hex_color, void *data, size_t length)
{
    if(map_ == nullptr)
        return;

    if(!trajectory_)
        trajectory_ = std::make_unique<PointCloud>(logger_, frame_width_, frame_height_, true);

    trajectory_->UseSharedTextures(map_->GetColorTexture());

    glm::vec3 color = HexToGLColor(hex_color);
    trajectory_->InitTrajectory(color, data, length);
}

void Map_Renderer::ClearTrajectory()
{
    trajectory_          = nullptr;
    external_trajectory_ = nullptr;
}

void Map_Renderer::ClearMap()
{
    map_          = nullptr;
    external_map_ = nullptr;
}

#pragma region Rendering
std::vector<uint8_t> Map_Renderer::Render(bool renderTrajectory,
                                          bool renderRobot,
                                          bool renderCamera)
{
    ZoneScoped;
    ZoneName("Map_Renderer::Render", 20);
    TracyGpuZone("GPU::RenderFrame");

    PointCloud *map_to_render = external_map_ ? external_map_ : map_.get();
    PointCloud *trajectory_to_render =
        external_trajectory_ ? external_trajectory_ : trajectory_.get();

    // update camera position if needed
    if(follow_robot_)
    {
        FollowRobotPosition();
    }

    // get camera transformation position and view
    glm::mat4 map_proj = camera_->GetMapProjectionMatrix();
    glm::mat4 hud_proj = camera_->GetHUDProjectionMatrix();

    glm::mat4 view = camera_->GetViewMatrix();

    glm::mat4 world_model(1.f);
    glm::mat4 robot_model = glm::mat4(1.f);

    // Orient robot view along the Z axis. Model png is drawn upwards and needs to be rotated an
    // additional 90 degrees
    glm::vec3 robot_position = {robot_->GetPosition(), 0.f};
    robot_model              = glm::translate(robot_model, robot_position);
    robot_model              = glm::rotate(robot_model, -robot_orientation_ + glm::radians(-90.f),
                                           glm::vec3(0.f, 0.f, 1.f));
    robot_model              = glm::translate(robot_model, -robot_position);

    // rotate the view around the robot
    if(follow_orientation_)
    {
        glm::vec3 camera_position = {camera_->GetPosition(), 0.f};
        view                      = glm::translate(view, camera_position);
        view                      = glm::rotate(view, robot_orientation_, glm::vec3(0, 0, 1));
        view                      = glm::translate(view, -camera_position);
    }

    // Clear the shared point cloud texture once before rendering any point clouds
    if(map_to_render != nullptr)
    {
        map_to_render->ClearTextures();
        map_to_render->Render(view, map_proj, world_model);
    }
    if(trajectory_to_render != nullptr && renderTrajectory)
        trajectory_to_render->Render(view, map_proj, world_model);

    // Draw fullscreen quad sampling color_texture
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glViewport(0, 0, frame_width_, frame_height_);

    // white background
    glClearColor(1.f, 1.f, 1.f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // render markers, robot and crosshair only when map and trajectory are valid
    if(map_to_render != nullptr || trajectory_to_render != nullptr)
    {
        glUseProgram(quadProg_);
        glActiveTexture(GL_TEXTURE0);
        if(map_to_render != nullptr)
            glBindTexture(GL_TEXTURE_2D, map_to_render->GetColorTexture());
        else if(trajectory_to_render != nullptr)
            glBindTexture(GL_TEXTURE_2D, trajectory_to_render->GetColorTexture());
        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        // Draw HUD elements after point clouds have been rendered
        float rotation_correction = camera_->GetRotationRadians();
        if(follow_orientation_)
        {
            rotation_correction -= robot_orientation_;
        }

        for(const auto &marker : markers_)
        {
            glm::mat4 marker_view = view;

            // view is centered around robot/camera. Each marker's view is centered around it's
            // position in the world.
            glm::vec3 pos = {marker.second->GetPosition(), 0.f};

            marker_view = glm::translate(marker_view, pos);
            marker_view = glm::rotate(marker_view, rotation_correction, glm::vec3(0, 0, 1));
            marker_view = glm::translate(marker_view, -pos);
            marker.second->Render(marker_view, hud_proj, world_model, true);
        }

        if(renderRobot)
        {
            robot_->Render(view, hud_proj, robot_model);
        }
        if(renderCamera)
        {
            glm::vec3 cam_pos = {camera_->GetPosition(), 0.f};

            view = glm::translate(view, cam_pos);
            view = glm::rotate(view, rotation_correction, glm::vec3(0, 0, 1));
            view = glm::translate(view, -cam_pos);

            camera_->Render(view, hud_proj);
        }

        // Render floor indicator in screen space (independent of zoom and camera position)
        glm::mat4 hud_view = glm::mat4(1.0f); // Identity matrix for screen-space rendering
        // Aspect-ratio corrected projection to avoid squashed text
        float aspect_ratio = static_cast<float>(frame_width_) / static_cast<float>(frame_height_);
        glm::mat4 screen_proj = glm::ortho(-aspect_ratio, aspect_ratio, -1.0f, 1.0f);
        // Position text in bottom-left corner with equal pixel margins
        float     margin_y             = 0.1f; // Same margin in both directions for equal pixels
        float     margin_x             = margin_y / aspect_ratio;
        glm::vec2 bottom_left_position = {-aspect_ratio + margin_x, 1.0f - margin_y};
        floor_indicator_->SetPosition(bottom_left_position);
        floor_indicator_->Render(hud_view, screen_proj);

        // Render scale indicator in screen space
        // Position it in the bottom-right corner with margins similar to floor indicator
        glm::vec2 bottom_right_position = {aspect_ratio - margin_x, 1.0f - margin_y};
        scale_indicator_->SetPosition(bottom_right_position);
        scale_indicator_->Render(hud_view, screen_proj, camera_->GetZoom(), aspect_ratio,
                                 frame_width_);
    }

    pixels_.resize(frame_width_ * frame_height_ * 3);
    glReadPixels(0, 0, frame_width_, frame_height_, GL_RGB, GL_UNSIGNED_BYTE, pixels_.data());

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Collect GPU profiling data at end of frame
    TracyGpuCollect;

    return pixels_;
}

void Map_Renderer::ResizeEGLSurface(int width, int height)
{
    frame_width_  = width;
    frame_height_ = height;

    PointCloud *map_to_resize = external_map_ ? external_map_ : map_.get();
    PointCloud *trajectory_to_resize =
        external_trajectory_ ? external_trajectory_ : trajectory_.get();

    // Delete color texture and GL buffers before destroying surface
    if(color_texture_ != 0)
        glDeleteTextures(1, &color_texture_);
    if(rbo_ != 0)
        glDeleteRenderbuffers(1, &rbo_);
    if(fbo_ != 0)
        glDeleteFramebuffers(1, &fbo_);

    // Unbind old surface
    eglMakeCurrent(eglDisplay_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    if(eglSurface_ != EGL_NO_SURFACE)
    {
        eglDestroySurface(eglDisplay_, eglSurface_);
    }

    const EGLint pbufferAttribs[] = {EGL_WIDTH, frame_width_, EGL_HEIGHT, frame_height_, EGL_NONE};

    eglSurface_ = eglCreatePbufferSurface(eglDisplay_, eglConfig_, pbufferAttribs);
    if(eglSurface_ == EGL_NO_SURFACE)
    {
        RCLCPP_ERROR(logger_, "Failed to recreate EGL pbuffer surface");
        return;
    }

    if(!eglMakeCurrent(eglDisplay_, eglSurface_, eglSurface_, eglContext_))
    {
        RCLCPP_ERROR(logger_, "Failed to make EGL context current after resize");
    }

    // Frame buffer object
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

    // Color texture
    glGenTextures(1, &color_texture_);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, frame_width_, frame_height_, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture_, 0);

    // Depth buffer
    glGenRenderbuffers(1, &rbo_);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, frame_width_, frame_height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo_);

    camera_->Resize(glm::vec2(frame_width_, frame_height_));

    if(map_to_resize != nullptr)
    {
        map_to_resize->Resize(frame_width_, frame_height_);
    }
    if(trajectory_to_resize != nullptr)
    {
        trajectory_to_resize->Resize(frame_width_, frame_height_);
    }
}

void Map_Renderer::GetExportResolution(int &width, int &height)
{
    // Use external map if available, otherwise use internal map
    PointCloud *active_map = external_map_ ? external_map_ : map_.get();

    // return default when no map is loaded yet
    if(active_map == nullptr)
    {
        width  = frame_width_;
        height = frame_height_;
        return;
    }

    PointCloud::MinMax x, y;
    active_map->GetMinMaxExportRaw(x, y);

    width  = std::max(0, static_cast<int>(std::ceil(x.max - x.min)));
    height = std::max(0, static_cast<int>(std::ceil(y.max - y.min)));
}

void Map_Renderer::GetExportCenter(float &x, float &y)
{
    // Use external map if available, otherwise use internal map
    PointCloud *active_map = external_map_ ? external_map_ : map_.get();

    if(active_map == nullptr)
    {
        x = 0.f;
        y = 0.f;
        return;
    }

    PointCloud::MinMax mm_x, mm_y;
    active_map->GetMinMaxExportRaw(mm_x, mm_y);

    x = (mm_x.max + mm_x.min) / 2.f;
    y = (mm_y.max + mm_y.min) / 2.f;
}

void Map_Renderer::GetCurrentReslution(int &width, int &height)
{
    width  = frame_width_;
    height = frame_height_;
}

// Function to convert a hex color string (#RRGGBBAA or RRGGBB) to normalized RGBA float values
glm::vec3 Map_Renderer::HexToGLColor(std::string value)
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

#pragma region camera
void Map_Renderer::UpdateCameraZoom(float zoom)
{
    camera_->Zoom(zoom);
}

void Map_Renderer::MoveCameraOnXAxis(float x)
{
    glm::vec2 delta               = {x, 0.f};
    float     rotation_correction = camera_->GetRotationRadians();
    if(follow_orientation_)
    {
        rotation_correction -= robot_orientation_;
    }
    delta = glm::rotate(glm::mat4(1.f), rotation_correction, glm::vec3(0.f, 0.f, 1.f)) *
            glm::vec4(delta, 0.f, 1.f);
    camera_->Move(delta);
    follow_robot_offset_ += delta;
}

void Map_Renderer::MoveCameraOnYAxis(float y)
{
    glm::vec2 delta               = {0.f, y};
    float     rotation_correction = camera_->GetRotationRadians();
    if(follow_orientation_)
    {
        rotation_correction -= robot_orientation_;
    }
    delta = glm::rotate(glm::mat4(1.f), rotation_correction, glm::vec3(0.f, 0.f, 1.f)) *
            glm::vec4(delta, 0.f, 1.f);

    camera_->Move(delta);
    follow_robot_offset_ += delta;
}

void Map_Renderer::RotateCameraAroundZAxis(float angle_deg)
{
    camera_->Rotate(angle_deg);
    follow_robot_rotation_offset_ += angle_deg;
}

float Map_Renderer::GetAbsRotation()
{
    return camera_->GetRotation();
}

void Map_Renderer::SetAbsRotation(float angle_deg)
{
    camera_->SetAbsRotation(angle_deg);
}

void Map_Renderer::GetCameraPosition(float &x, float &y)
{
    glm::vec2 position = camera_->GetPosition();
    x                  = position.x;
    y                  = position.y;
}

void Map_Renderer::GetCameraPose(glm::vec2 &position, glm::vec4 &orientation)
{
    // camera position is in render-world coordinates, which is apparently different from real-world
    // coordinates... differences: rotation offset of 90 deg (0 deg in real = left in render)
    // rotation inverse (-90 deg in real = up in render, 90 deg = down in render)
    // right in real (+y) = up in render (-y)
    // forward in real (-x) = left in render (-x)
    // ...
    // looks like coordinate systems are the same except that rotation is performed different
    // render: default y = up, rotate clockwise
    // real: default x = down, rotate counterclockwise
    // simply applying 90deg offset and inversing rotation should be enough
    // also invert y
    position    = camera_->GetPosition();
    position.y  = -position.y;
    float angle = camera_->GetRotation();

    // 90deg offset
    angle += 90.f;
    // invert and convert to radians
    angle = glm::radians(-angle);
    // (ROS2) Quaternion from angle rotation around Z axis
    orientation = glm::vec4(0.f, 0.f, sin(angle / 2.f), cos(angle / 2.f));
}

void Map_Renderer::SetCameraPosition(float x, float y)
{
    camera_->SetPosition(glm::vec2(x, y) + follow_robot_offset_);
}

void Map_Renderer::SetAbsCameraPosition(float x, float y)
{
    camera_->SetPosition(glm::vec2(x, y));
}


#pragma region marker

void Map_Renderer::AddMarker(const int index, const std::string label, float x, float y)
{
    if(markers_.count(index))
        return;

    auto new_marker =
        std::make_shared<Marker>(logger_, hud_texture_, text_renderer_, glm::vec2(x, y), label);

    markers_.insert(std::pair<int, std::shared_ptr<Marker>>(index, new_marker));
}

void Map_Renderer::RemoveMarker(const int index)
{
    markers_.erase(index);
}

void Map_Renderer::RemoveAllMarkers()
{
    markers_.clear();
}

void Map_Renderer::UpdateMarkerLabel(const int index, const std::string label)
{
    if(markers_.count(index))
        markers_.at(index)->UpdateLabel(label);
}

#pragma region Robot position

void Map_Renderer::UpdateRobotPosition(const float x, const float y)
{
    robot_->UpdatePosition(glm::vec2(x, y));
}

void Map_Renderer::ResetRobotPosition()
{
    robot_->ResetPosition();
}

void Map_Renderer::UpdateRobotOrientation(const float z)
{
    // Rotate along Z axis.
    // X and Y axis rotation do not exists in 2D space
    robot_orientation_ = z;
}

void Map_Renderer::MoveCameraToRobotPosition()
{
    follow_robot_offset_ = glm::vec2(0.f, 0.f);
    camera_->SetPosition(robot_->GetPosition());
}

void Map_Renderer::FollowRobotPosition()
{
    camera_->SetPosition(robot_->GetPosition() + follow_robot_offset_);
}

void Map_Renderer::FollowRobotOff()
{
    follow_robot_                 = false;
    follow_orientation_           = false;
    follow_robot_offset_          = glm::vec2(0.f, 0.f);
    follow_robot_rotation_offset_ = 0.f;
}

void Map_Renderer::FollowRobotOn(bool with_orientation)
{
    follow_robot_        = true;
    follow_orientation_  = with_orientation;
    follow_robot_offset_ = camera_->GetPosition() - robot_->GetPosition();
}

void Map_Renderer::GetFollowRobotStatus(bool &is_following, bool &is_following_orientation)
{
    is_following             = follow_robot_;
    is_following_orientation = follow_orientation_;
}

void Map_Renderer::SetFloorName(const std::string &floor_name)
{
    if(floor_indicator_)
    {
        floor_indicator_->SetFloorName(floor_name);
    }
}

std::string Map_Renderer::GetFloorName() const
{
    if(floor_indicator_)
    {
        return floor_indicator_->GetFloorName();
    }
    return "";
}

void Map_Renderer::SetScaleUnitMeters(bool use_meters)
{
    if(scale_indicator_)
    {
        scale_indicator_->SetUseMeters(use_meters);
    }
}
