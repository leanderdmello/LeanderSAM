// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula


#pragma once

#include <vector>
#include <string>
#include <atomic>

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <rclcpp/rclcpp.hpp>

// 16-byte alignment for std430 SSBO vec4 compatibility:
//  https://community.khronos.org/t/ssbo-std430-layout-rules/109761
struct alignas(16) Point
{
    float x, y, z, reflect;
};

// Cant use alignas as it forces memory allignment, makeing the struct 8 bytes bigger.
// have to pragma pack to force 36UL 4 bute allignment, else 40UL is compiled
#pragma pack(push, 4)
struct PointPRYT
{
    float  x, y, z, reflect;
    float  pitch;
    float  rol;
    float  yaw;
    double time;
};
#pragma pack(pop)

class PointCloud
{
public:
    struct MinMax
    {
        float min;
        float max;
    };

    struct LayeredMap
    {
        float        resolution;
        glm::i32vec3 dim;
        MinMax       x_minmax;
        MinMax       y_minmax;
        MinMax       z_minmax;
        float        voxel_size;
        // number of layers * points per layer
        std::vector<std::vector<Point>> layer;
    };

    enum class Mode
    {
        Map,
        Trajectory
    };

    PointCloud(const rclcpp::Logger &logger, int width, int height, bool use_shared_textures);
    ~PointCloud();

    void InitTrajectory(glm::vec3 color, const std::string uri);
    void InitTrajectory(glm::vec3 color, void *data, size_t length);
    void InitMap(const std::string uri);
    void InitMap(void *data, size_t length);

    // Split initialization: CPU preprocessing and GPU upload
    void PreprocessMap(const std::string uri);
    void PreprocessMap(void *data, size_t length);
    void PreprocessTrajectory(glm::vec3 color, const std::string uri);
    void PreprocessTrajectory(glm::vec3 color, void *data, size_t length);
    void UploadToGPU(Mode mode); // Must be called on render thread

    GLuint GetColorTexture() const;

    void GetMinMaxExport(MinMax &x, MinMax &y);
    void GetMinMaxExportRaw(MinMax &x, MinMax &y);

    void Resize(int newWidth, int newHeight);
    void Render(const glm::mat4 &view, const glm::mat4 &proj, const glm::mat4 &model);

    // Used for shared rendering
    void UseSharedTextures(GLuint sharedColor);

    // Texture management - exposed for external control of clear timing
    void ClearTextures();

    // Introspection
    size_t GetPointCount() const { return pc_data_.size(); }

private:
    void CreateTextures();
    void Init(const char *shader_source, Mode mode);
    int  LoadPointCloudFromFile(const std::string uri, Mode mode);
    int  LoadPointCloudFromMemory(void *data, size_t length);

    // CPU preprocessing - can run on any thread
    void PreprocessRawData(Mode mode);

    // Point cloud data manipulation
    std::vector<Point> PreFilterZ(const std::vector<Point> &pointcloud, float min_z, float max_z);
    LayeredMap         PointCloudToLayedMap(const std::vector<Point> &pointcloud,
                                            float                     resolution     = 0.1f,
                                            bool                      flatten_z_axis = false);
    LayeredMap         SliceRows(const LayeredMap &data, const size_t from, const size_t to);
    LayeredMap         Voxelize(const LayeredMap &data);
    std::vector<Point> LayeredMapToPointcloud(const LayeredMap &data);
    void               FlipYAxis(std::vector<Point> &pointcloud);

    std::vector<Point> pc_data_;
    GLuint             ssbo_            = 0;
    GLuint             color_texture_   = 0;
    GLuint             compute_program_ = 0;

    std::vector<uint8_t> color_init_;

    glm::vec3 color_{1.f, 1.f, 1.f};

    int width_  = 0;
    int height_ = 0;

    float resolution_ = 0.05f;

    bool use_shared_textures_ = false;

    // Tracks whether CPU preprocessing finished before GPU upload
    std::atomic<bool> preprocessed_ready_{false};

    // Uniforms
    GLint          loc_num_points_;
    GLint          loc_model_;
    GLint          loc_view_;
    GLint          loc_proj_;
    GLint          loc_viewport_;
    GLint          loc_voxel_size_;
    GLint          loc_color_;
    rclcpp::Logger logger_;
};
