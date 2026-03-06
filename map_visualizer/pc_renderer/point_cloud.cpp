// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "point_cloud.h"

#include <tracy/Tracy.hpp>
#include <tracy/TracyOpenGL.hpp>

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <chrono>

#include "shader.h"
#include "point_cloud_shaders.h"

#include <stack>

#pragma region KDTREE

// Simple KD tree
class KDTree
{
    struct Node
    {
        int   index; // index into points array (the splitting point)
        int   axis;  // 0=x,1=y,2=z
        float split; // split coordinate value (points[index].x/y/z)
        int   left;  // child index in nodes vector or -1
        int   right; // child index in nodes vector or -1
    };

    const std::vector<Point> *points;
    std::vector<int>          idx; // permutation of point indices used for nth_element
    std::vector<Node>         nodes;

public:
    KDTree(const std::vector<Point> &pts) : points(&pts)
    {
        idx.resize(pts.size());
        for(size_t i = 0; i < pts.size(); ++i)
            idx[i] = (int)i;
        nodes.reserve(pts.size());
        if(!pts.empty())
            build(0, (int)idx.size(), 0);
    }

    void radiusSearch(const glm::vec3 &query, float radius2, std::vector<int> &result) const
    {
        result.clear();
        if(nodes.empty())
            return;

        std::stack<int> stack;
        stack.push(0); // root

        while(!stack.empty())
        {
            int ni = stack.top();
            stack.pop();
            const Node  &node = nodes[ni];
            const Point &p    = (*points)[node.index];

            float dx    = query.x - p.x;
            float dy    = query.y - p.y;
            float dz    = query.z - p.z;
            float dist2 = dx * dx + dy * dy + dz * dz;
            if(dist2 <= radius2)
                result.push_back(node.index);

            // choose branch
            float diff =
                (node.axis == 0 ? query.x - node.split :
                                  (node.axis == 1 ? query.y - node.split : query.z - node.split));

            int near_child = (diff <= 0.0f) ? node.left : node.right;
            int far_child  = (diff <= 0.0f) ? node.right : node.left;

            if(near_child >= 0)
                stack.push(near_child);
            if(far_child >= 0 && diff * diff <= radius2)
                stack.push(far_child);
        }
    }

private:
    // build between [l, r) in idx array. returns node index in nodes vector
    int build(int l, int r, int depth)
    {
        if(l >= r)
            return -1;

        int axis = depth % 3;
        int mid  = (l + r) >> 1;

        auto cmp = [&](int a, int b)
        {
            if(axis == 0)
                return (*points)[a].x < (*points)[b].x;
            if(axis == 1)
                return (*points)[a].y < (*points)[b].y;

            return (*points)[a].z < (*points)[b].z;
        };

        std::nth_element(idx.begin() + l, idx.begin() + mid, idx.begin() + r, cmp);

        Node node;
        node.index = idx[mid];
        node.axis  = axis;
        node.split = (axis == 0 ? (*points)[node.index].x :
                                  (axis == 1 ? (*points)[node.index].y : (*points)[node.index].z));
        node.left = node.right = -1;

        int myIndex = (int)nodes.size();
        nodes.push_back(node);

        nodes[myIndex].left = build(l, mid, depth + 1);
        ;
        nodes[myIndex].right = build(mid + 1, r, depth + 1);
        ;

        return myIndex;
    }
};

#pragma endregion KDTREE
#pragma region vertical_cassification

// PCA smallest eigenvector estimation
glm::vec3 SmallestEigenVector(const glm::mat3 &A, int maxIter = 64, float tol = 1e-6f)
{
    // Regularize to avoid exact singularity
    glm::mat3 reg  = A + glm::mat3(1e-8f);
    glm::mat3 invA = glm::inverse(reg);
    glm::vec3 v(1.0f, 1.0f, 1.0f);
    v = glm::normalize(v);
    glm::vec3 vNext;
    for(int i = 0; i < maxIter; ++i)
    {
        vNext   = invA * v;
        float n = glm::length(vNext);
        if(n <= 1e-12f)
            break;
        vNext /= n;
        if(glm::length(vNext - v) < tol)
        {
            v = vNext;
            break;
        }

        v = vNext;
    }
    // ensure normalization
    float ln = glm::length(v);
    if(ln > 0.0f)
        v /= ln;

    return v;
}

// Main classifier: see https://www.sciencedirect.com/science/article/pii/S1569843224006216
std::pair<std::vector<Point>, std::vector<Point>> ClassifyHorizontalVertical(
    const std::vector<Point> &cloud,
    float                     neighbor_radius         = 0.50f,
    float                     theta_vertical_degree   = 5.0f,
    float                     theta_horizontal_degree = 5.0f)
{
    std::vector<Point> vertical_points;
    std::vector<Point> horizontal_points;
    if(cloud.empty())
        return {horizontal_points, vertical_points};

    KDTree           kdtree(cloud);
    std::vector<int> neighbors;
    neighbors.reserve(64);

    // Z axis is the vertical direction
    const glm::vec3 z_axis(0.0f, 0.0f, 1.0f);

    const float r2           = neighbor_radius * neighbor_radius;
    const float t_vertical   = glm::radians(theta_vertical_degree);
    const float t_horizontal = glm::radians(theta_horizontal_degree);
    const float t_ninety     = glm::radians(90.f);

    for(size_t i = 0; i < cloud.size(); ++i)
    {
        glm::vec3 query(cloud[i].x, cloud[i].y, cloud[i].z);
        kdtree.radiusSearch(query, r2, neighbors);
        if(neighbors.size() < 3)
            continue;

        // Compute centroid value
        glm::vec3 centroid(0.0f);
        for(int idx : neighbors)
            centroid += glm::vec3(cloud[idx].x, cloud[idx].y, cloud[idx].z);

        centroid /= static_cast<float>(neighbors.size());

        // Covariance matrix
        glm::mat3 cov(0.0f);
        for(int idx : neighbors)
        {
            glm::vec3 d = glm::vec3(cloud[idx].x, cloud[idx].y, cloud[idx].z) - centroid;
            cov[0][0] += d.x * d.x;
            cov[0][1] += d.x * d.y;
            cov[0][2] += d.x * d.z;
            cov[1][0] += d.y * d.x;
            cov[1][1] += d.y * d.y;
            cov[1][2] += d.y * d.z;
            cov[2][0] += d.z * d.x;
            cov[2][1] += d.z * d.y;
            cov[2][2] += d.z * d.z;
        }
        cov /= static_cast<float>(neighbors.size());

        glm::vec3 normal = SmallestEigenVector(cov);

        // Classify aligned with Y (vertical)
        float dot_product = glm::clamp(glm::dot(normal, z_axis), -1.0f, 1.0f);
        float angle       = std::acos(dot_product);

        if(angle <= t_horizontal)
        {
            horizontal_points.push_back(cloud[i]);
        }
        else if(fabs(angle - t_ninety) <= t_vertical)
        {
            vertical_points.push_back(cloud[i]);
        }
    }

    return {std::move(horizontal_points), std::move(vertical_points)};
}

#pragma endregion vertical_cassification

PointCloud::PointCloud(const rclcpp::Logger &logger,
                       int                   width,
                       int                   height,
                       bool                  use_shared_textures)
    : logger_(logger), width_(width), height_(height), use_shared_textures_(use_shared_textures)
{
    // Textures will be created in UploadToGPU (on render thread) when needed
}

PointCloud::~PointCloud()
{
    if(ssbo_ != 0)
        glDeleteBuffers(1, &ssbo_);

    if(!use_shared_textures_)
        glDeleteTextures(1, &color_texture_);

    if(compute_program_ != 0)
        glDeleteProgram(compute_program_);
}

void PointCloud::InitTrajectory(glm::vec3 color, const std::string uri)
{
    color_ = color;
    if(LoadPointCloudFromFile(uri, Mode::Trajectory) != 0)
        exit(1);

    Init(compute_shader_src, Mode::Trajectory);
}

void PointCloud::InitTrajectory(glm::vec3 color, void *data, size_t length)
{
    color_ = color;
    if(LoadPointCloudFromMemory(data, length) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load point cloud data from memory");
        return;
    }

    Init(compute_shader_src, Mode::Trajectory);
}

void PointCloud::InitMap(const std::string uri)
{
    if(LoadPointCloudFromFile(uri, Mode::Map) != 0)
        exit(1);

    Init(compute_shader_src, Mode::Map);
}

void PointCloud::InitMap(void *data, size_t length)
{
    if(LoadPointCloudFromMemory(data, length) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load point cloud data from memory");
        return;
    }

    Init(compute_shader_src, Mode::Map);
}

void PointCloud::PreprocessMap(const std::string uri)
{
    ZoneScoped;
    ZoneName("PreprocessMap::File", 19);

    if(LoadPointCloudFromFile(uri, Mode::Map) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load map from file: %s", uri.c_str());
        return;
    }
    // CPU preprocessing happens here in PreprocessRawData
    PreprocessRawData(Mode::Map);
}

void PointCloud::PreprocessMap(void *data, size_t length)
{
    ZoneScoped;
    ZoneName("PreprocessMap::Memory", 21);
    TracyPlot("MapDataSize", (int64_t)length);

    if(LoadPointCloudFromMemory(data, length) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load point cloud data from memory");
        return;
    }
    PreprocessRawData(Mode::Map);
}

void PointCloud::PreprocessTrajectory(glm::vec3 color, const std::string uri)
{
    ZoneScoped;
    ZoneName("PreprocessTrajectory::File", 26);

    color_ = color;
    if(LoadPointCloudFromFile(uri, Mode::Trajectory) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load trajectory from file: %s", uri.c_str());
        return;
    }
    PreprocessRawData(Mode::Trajectory);
}

void PointCloud::PreprocessTrajectory(glm::vec3 color, void *data, size_t length)
{
    ZoneScoped;
    ZoneName("PreprocessTrajectory::Memory", 28);
    TracyPlot("TrajectoryDataSize", (int64_t)length);

    color_ = color;
    if(LoadPointCloudFromMemory(data, length) != 0)
    {
        RCLCPP_ERROR(logger_, "Failed to load point cloud data from memory");
        return;
    }
    PreprocessRawData(Mode::Trajectory);
}

void PointCloud::PreprocessRawData(Mode mode)
{
    // This is the CPU-intensive preprocessing that can happen off the render thread
    if(mode == Mode::Map)
    {
        ZoneScoped;
        ZoneName("PreprocessRawData::Map", 18);
        // manipulate cloud
        pc_data_ = PreFilterZ(pc_data_, 0.0f, 1.0f);

        // TEMPORARILY SKIP: ClassifyHorizontalVertical takes very long time for even small clouds
        // TODO: Fix speed or find alternative
        // auto classy = ClassifyHorizontalVertical(pc_data_, 0.5f, 2.f, 3.f);
        // pc_data_ = classy.second;

        auto layered_cloud = PointCloudToLayedMap(pc_data_, resolution_, true);

        float n_layers    = static_cast<float>(layered_cloud.layer.size());
        int   lower_bound = static_cast<int>(n_layers * 0.0f); // bottom 0%
        int   upper_bound = static_cast<int>(n_layers * 1.0f); // top 100%

        auto sliced = SliceRows(layered_cloud, lower_bound, upper_bound);

        // voxelize
        auto voxel_layer = Voxelize(sliced);

        // Just enforce one size for voxels
        if(voxel_layer.voxel_size < 2.f * resolution_ && voxel_layer.voxel_size > 0.f)
        {
            resolution_ = voxel_layer.voxel_size;
        }
        else
        {
            RCLCPP_ERROR(logger_, "resolution grew bigger than 2 * resolution: 0.1 vs %.3f",
                         voxel_layer.voxel_size);
        }

        auto cloud = LayeredMapToPointcloud(voxel_layer);
        pc_data_   = cloud;
    }

    // Invert Y axis
    FlipYAxis(pc_data_);

    // Mark preprocessing complete; UploadToGPU may now be called
    preprocessed_ready_ = true;
}

void PointCloud::UploadToGPU(Mode mode)
{
    ZoneScoped;
    ZoneName("UploadToGPU", 11);
    TracyGpuZone("GPU::UploadPointCloud");

    if(!preprocessed_ready_)
    {
        RCLCPP_ERROR(
            logger_,
            "UploadToGPU called before preprocessing completed; call PreprocessRawData first");
        return;
    }

    // This method contains ONLY OpenGL operations and must run on render thread
    TracyPlot("PointCloudSize", (int64_t)pc_data_.size());

    // Create texture if not using shared textures and not already created
    if(!use_shared_textures_ && color_texture_ == 0)
    {
        CreateTextures();
    }

    // Delete resources on re-init
    if(ssbo_ != 0)
        glDeleteBuffers(1, &ssbo_);
    if(compute_program_ != 0)
        glDeleteProgram(compute_program_);

    // Create compute program
    GLuint cs        = compileShader(logger_, GL_COMPUTE_SHADER, compute_shader_src);
    compute_program_ = linkProgramFromShaders(logger_, {cs});

    // catch empty cloud
    if(pc_data_.empty())
    {
        ssbo_ = 0;
        return;
    }

    // Upload point data to GPU
    glGenBuffers(1, &ssbo_);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_);
    glBufferData(GL_SHADER_STORAGE_BUFFER, pc_data_.size() * sizeof(Point), pc_data_.data(),
                 GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_);

    glUseProgram(compute_program_);
    loc_num_points_ = glGetUniformLocation(compute_program_, "u_num_points");
    loc_model_      = glGetUniformLocation(compute_program_, "u_model");
    loc_view_       = glGetUniformLocation(compute_program_, "u_view");
    loc_proj_       = glGetUniformLocation(compute_program_, "u_proj");
    loc_viewport_   = glGetUniformLocation(compute_program_, "u_viewport");
    loc_voxel_size_ = glGetUniformLocation(compute_program_, "u_voxel_size");
    loc_color_      = glGetUniformLocation(compute_program_, "u_color");

    glUniform1ui(loc_num_points_, (GLuint)pc_data_.size());
    glUniform2i(loc_viewport_, width_, height_);
    glUniform1f(loc_voxel_size_, resolution_);

    if(mode == Mode::Trajectory)
    {
        glUniform3f(loc_color_, color_.r, color_.g, color_.b);
    }

    if(mode == Mode::Map)
    {
        // Color map gray
        color_.r = 0.6f;
        color_.g = 0.6f;
        color_.b = 0.6f;
        glUniform3f(loc_color_, color_.r, color_.g, color_.b);
    }
}

void PointCloud::Init(const char *shader_source, Mode mode)
{
    // This is the legacy method that does everything
    // New code should use PreprocessMap/PreprocessTrajectory + UploadToGPU
    PreprocessRawData(mode);
    UploadToGPU(mode);
}

void PointCloud::CreateTextures()
{
    // all white
    color_init_.resize(width_ * height_ * 4, 255);

    glGenTextures(1, &color_texture_);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width_, height_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

GLuint PointCloud::GetColorTexture() const
{
    return color_texture_;
}

void PointCloud::ClearTextures()
{
    // Let caller handle clearing
    if(use_shared_textures_)
        return;

    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGBA, GL_UNSIGNED_BYTE,
                    color_init_.data());
}

int PointCloud::LoadPointCloudFromFile(const std::string uri, Mode mode)
{
    std::ifstream file(uri);
    if(!file)
    {
        RCLCPP_ERROR(logger_, "Error: could not open file: %s", uri.c_str());
        return -1;
    }

    std::string token;
    size_t      point_count = 0;
    bool        pointsFound = false, dataFound = false;

    // Get n points and check for data
    while(file >> token)
    {
        if(token == "POINTS")
        {
            if(!(file >> point_count))
            {
                RCLCPP_ERROR(logger_, "Error: invalid POINTS value");
                return -1;
            }
            pointsFound = true;
        }
        else if(token == "DATA")
        {
            dataFound = true;
            // Move to the next line before reading binary data
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            break;
        }
    }

    if(!pointsFound || !dataFound)
    {
        RCLCPP_ERROR(logger_, "Error: missing POINTS or DATA section in file: %s", uri.c_str());
        return -1;
    }

    // Read data field, trajectory has extra fields
    if(mode == Mode::Trajectory)
    {
        // trajectory file isnt too big
        std::vector<PointPRYT> tmp;
        tmp.resize(point_count);
        file.read(reinterpret_cast<char *>(tmp.data()), point_count * sizeof(PointPRYT));

        pc_data_.resize(point_count);
        for(size_t i = 0; i < tmp.size(); i++)
        {
            pc_data_[i].x       = tmp[i].x;
            pc_data_[i].y       = tmp[i].y;
            pc_data_[i].z       = tmp[i].z;
            pc_data_[i].reflect = tmp[i].reflect;
        }
    }
    else
    {
        pc_data_.resize(point_count);
        file.read(reinterpret_cast<char *>(pc_data_.data()), point_count * sizeof(Point));
    }

    if(!file)
    {
        RCLCPP_ERROR(logger_, "Error: unexpected end of file while reading data from: %s",
                     uri.c_str());
        return -1;
    }
    RCLCPP_DEBUG(logger_, "Read %zu points", pc_data_.size());

    // Reset preprocessing state for freshly loaded data
    preprocessed_ready_ = false;

    return 0;
}

int PointCloud::LoadPointCloudFromMemory(void *data, size_t length)
{
    ZoneScoped;
    ZoneName("LoadPointCloudFromMemory", 26);
    // Read data field
    pc_data_.resize(length / sizeof(Point));
    std::memcpy(pc_data_.data(), data, length);
    preprocessed_ready_ = false;
    return 0;
}

void PointCloud::Resize(int newWidth, int newHeight)
{
    if(newWidth == width_ && newHeight == height_)
        return;

    width_  = newWidth;
    height_ = newHeight;

    if(!use_shared_textures_)
    {
        glDeleteTextures(1, &color_texture_);
        CreateTextures();
    }

    glUseProgram(compute_program_);
    glUniform2i(loc_viewport_, width_, height_);
}

void PointCloud::Render(const glm::mat4 &view, const glm::mat4 &proj, const glm::mat4 &model)
{
    if(pc_data_.empty() || ssbo_ == 0)
    {
        RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared(), 5000,
                             "PointCloud::Render skipped: empty or no SSBO (points=%zu, ssbo=%u)",
                             pc_data_.size(), ssbo_);
        return;
    }

    glUseProgram(compute_program_);

    glUniformMatrix4fv(loc_model_, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(loc_view_, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(loc_proj_, 1, GL_FALSE, glm::value_ptr(proj));

    // bind this cloud's SSBO to the binding index expected by the compute shader
    // so the shader reads the correct point data.
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_);

    glBindImageTexture(0, color_texture_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);

    const GLuint local_size = 256u;
    GLuint       groups     = (GLuint)((pc_data_.size() + local_size - 1) / local_size);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);

    GLenum err = glGetError();
    if(err != GL_NO_ERROR)
    {
        RCLCPP_ERROR(logger_, "OpenGL error after compute dispatch: 0x%x", err);
    }
}

void PointCloud::UseSharedTextures(GLuint sharedColor)
{
    color_texture_       = sharedColor;
    use_shared_textures_ = true;
}

void PointCloud::GetMinMaxExport(MinMax &x, MinMax &y)
{
    MinMax raw_x, raw_y;

    GetMinMaxExportRaw(raw_x, raw_y);

    // scale back
    x = {raw_x.min / resolution_, raw_x.max / resolution_};
    y = {raw_y.min / resolution_, raw_y.max / resolution_};
}

void PointCloud::GetMinMaxExportRaw(MinMax &x, MinMax &y)
{
    if(pc_data_.empty())
    {
        x = {0.f, static_cast<float>(width_)};
        y = {0.f, static_cast<float>(height_)};
        return;
    }

    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;

    for(const auto &point : pc_data_)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    x = {min_x, max_x};
    y = {min_y, max_y};
}

#pragma region PointCloudManip

std::vector<Point> PointCloud::PreFilterZ(const std::vector<Point> &pointcloud,
                                          float                     min_z,
                                          float                     max_z)
{
    std::vector<Point> filtered;

    for(auto point : pointcloud)
    {
        if(point.z >= min_z && point.z <= max_z)
        {
            filtered.push_back(point);
        }
    }

    return filtered;
}

PointCloud::LayeredMap PointCloud::PointCloudToLayedMap(const std::vector<Point> &pointcloud,
                                                        float                     resolution,
                                                        bool                      flatten_z_axis)
{
    LayeredMap map;
    if(pointcloud.empty())
        return map;

    if(resolution <= 0.f)
    {
        RCLCPP_ERROR(logger_, "PointCloudToLayedMap: invalid resolution <= 0");
        return map;
    }

    map.resolution = resolution;

    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    float min_z = FLT_MAX, max_z = -FLT_MAX;

    for(const auto &point : pointcloud)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }

    map.dim.x = std::max(0, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1);
    map.dim.y = std::max(0, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1);
    map.dim.z = std::max(0, static_cast<int>(std::ceil((max_z - min_z) / resolution)) + 1);

    map.x_minmax = {min_x, max_x};
    map.y_minmax = {min_y, max_y};
    map.z_minmax = {min_z, max_z};

    map.voxel_size = (max_z - min_z) / static_cast<float>(map.dim.z);

    map.layer.resize(map.dim.z);

    for(const auto &p : pointcloud)
    {
        int iz = (int)((p.z - min_z) / resolution);
        if(iz >= 0 && iz < map.dim.z)
        {
            map.layer.at(iz).push_back({p.x, p.y, (flatten_z_axis) ? 0 : p.z, p.reflect});
        }
    }

    // erase empty layers. (can also be a min nr of points)
    map.layer.erase(std::remove_if(map.layer.begin(), map.layer.end(),
                                   [](std::vector<Point> points) { return points.size() == 0; }),
                    map.layer.end());

    return map;
}

PointCloud::LayeredMap PointCloud::SliceRows(const LayeredMap &data,
                                             const size_t      from,
                                             const size_t      to)
{
    LayeredMap map;

    if(from >= to)
        return map;

    size_t min = std::min(from, data.layer.size());
    size_t max = std::min(to, data.layer.size());

    map.layer.resize(max - min);
    map.resolution = data.resolution;
    map.dim        = data.dim;
    map.voxel_size = data.voxel_size;
    map.x_minmax   = data.x_minmax;
    map.y_minmax   = data.y_minmax;
    map.z_minmax   = data.z_minmax;

    for(size_t i = min; i < max; i++)
    {
        map.layer.at(i - min) = data.layer.at(i);
    }

    return map;
}

std::vector<Point> PointCloud::LayeredMapToPointcloud(const LayeredMap &data)
{
    std::vector<Point> points;

    for(const auto &layer : data.layer)
        for(const auto &point : layer)
            points.push_back(point);

    return points;
}

PointCloud::LayeredMap PointCloud::Voxelize(const LayeredMap &data)
{
    LayeredMap map;

    if(data.layer.size() == 0)
        return map;

    // return empty is dim x or y is invalid
    if(data.dim.x == 0 || data.dim.y == 0)
        return map;

    map.resolution = data.resolution;
    map.dim        = data.dim;
    map.x_minmax   = data.x_minmax;
    map.y_minmax   = data.y_minmax;
    map.z_minmax   = data.z_minmax;
    map.voxel_size = data.voxel_size;
    map.layer.resize(data.layer.size());

    // Calculate voxel sizes for each dimension
    float voxel_size_x = (data.x_minmax.max - data.x_minmax.min) / static_cast<float>(data.dim.x);
    float half_x       = voxel_size_x * 0.5f;

    float voxel_size_y = (data.y_minmax.max - data.y_minmax.min) / static_cast<float>(data.dim.y);
    float half_y       = voxel_size_y * 0.5f;

    float voxel_size_z = (data.z_minmax.max - data.z_minmax.min) / static_cast<float>(data.dim.z);
    float half_z       = voxel_size_z * 0.5f;

    // take the largest
    map.voxel_size = std::max(voxel_size_x, map.voxel_size);
    map.voxel_size = std::max(voxel_size_y, map.voxel_size);

    // Process each layer with deduplication
    for(size_t i = 0; i < data.layer.size(); i++)
    {
        // Use map to deduplicate: key = grid cell (ix, iy) - iz not needed as each layer is already
        // Z-sliced
        std::map<std::pair<int, int>, Point> voxel_map;

        for(const auto &point : data.layer.at(i))
        {
            // Calculate grid coordinates
            int ix = static_cast<int>(std::round((point.x - data.x_minmax.min) / voxel_size_x));
            int iy = static_cast<int>(std::round((point.y - data.y_minmax.min) / voxel_size_y));
            int iz = static_cast<int>(std::round((point.z - data.z_minmax.min) / voxel_size_z));

            // Calculate snapped positions to voxel grid centers
            float snapped_x = ix * voxel_size_x + data.x_minmax.min + half_x;
            float snapped_y = iy * voxel_size_y + data.y_minmax.min + half_y;
            float snapped_z = iz * voxel_size_z + data.z_minmax.min + half_z;

            // Only keep first point per voxel (deduplicating on X,Y within this Z-layer)
            auto key = std::make_pair(ix, iy);
            if(voxel_map.find(key) == voxel_map.end())
            {
                voxel_map[key] = {snapped_x, snapped_y, snapped_z, point.reflect};
            }
        }

        // Extract deduplicated points
        for(const auto &kv : voxel_map)
        {
            map.layer.at(i).push_back(kv.second);
        }
    }
    return map;
}

void PointCloud::FlipYAxis(std::vector<Point> &pointcloud)
{
    for(auto &point : pointcloud)
    {
        point.y = -point.y;
    }
}

#pragma end region PointCloudManip
