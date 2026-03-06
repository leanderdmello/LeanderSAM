// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula


// Base color is always green and background is gray when texture color is gleared.
// We can move shader source to a file. This would only be usefull if you want to try
// different shaders during runtime without re-compiling source.

const char* compute_shader_src = R"(#version 430 core
layout(local_size_x = 256, local_size_y = 1, local_size_z = 1) in;

struct Point {
    vec4 pos; // x,y,z,reflect
};

layout(std430, binding = 0) readonly buffer Points {
    Point pts[];
};

uniform uint  u_num_points;
uniform mat4  u_view;
uniform mat4  u_proj;
uniform mat4  u_model;
uniform ivec2 u_viewport; // width, height
uniform float u_voxel_size; // world-space voxel size
uniform vec3  u_color;

layout(rgba8, binding = 0) writeonly uniform image2D imgColor;

// Ray - AABB intersection (slabs method).
// Returns true if ray intersects [boxMin, boxMax]. tminOut is near hit, tmaxOut far hit.
bool intersectAABB(in vec3 rayOrig, in vec3 rayDir, in vec3 boxMin, in vec3 boxMax,
                   out float tminOut, out float tmaxOut)
{
    float tmin = -1e30;
    float tmax =  1e30;

    // For each axis, compute slab intersection
    for (int i = 0; i < 3; ++i) {
        float ro = rayOrig[i];
        float rd = rayDir[i];
        float bmin = boxMin[i];
        float bmax = boxMax[i];

        if (abs(rd) < 1e-8) {
            // Parallel to slab: origin must be inside slab
            if (ro < bmin || ro > bmax) return false;
        } else {
            float invd = 1.0 / rd;
            float t1 = (bmin - ro) * invd;
            float t2 = (bmax - ro) * invd;
            if (t1 > t2) {
                float tmp = t1; t1 = t2; t2 = tmp;
            }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
            if (tmax < 0.0) return false; // fully behind the ray
        }
    }

    tminOut = tmin;
    tmaxOut = tmax;
    return true;
}

void main() {
    uint gid = gl_GlobalInvocationID.x;
    if (gid >= u_num_points) return;

    // Precompute common matrices per invocation
    mat4 mv     = u_view * u_model;
    mat4 invProj = inverse(u_proj);
    mat4 invMV   = inverse(mv); // transforms from view space to model space

    // model-space center of voxel
    vec3 p_model = pts[gid].pos.xyz;
    float half_voxel_size = u_voxel_size * 0.5;

    // compute AABB in model space
    vec3 boxMin = p_model - vec3(half_voxel_size);
    vec3 boxMax = p_model + vec3(half_voxel_size);

    // Build 8 corners via bit tricks and project to clip/NDC to get pixel bbox
    vec2 minPix = vec2(1e30);
    vec2 maxPix = vec2(-1e30);
    bool anyCornerInNDC = false;

    for (int c = 0; c < 8; ++c) {
        vec3 offs = vec3(
            ( (c & 1) != 0 ) ? half_voxel_size : -half_voxel_size,
            ( (c & 2) != 0 ) ? half_voxel_size : -half_voxel_size,
            ( (c & 4) != 0 ) ? half_voxel_size : -half_voxel_size
        );
        vec4 worldCorner = u_model * vec4(p_model + offs, 1.0);
        vec4 clip = u_proj * (u_view * worldCorner);

        // skip this corner if it's behind the clip (don't abort whole voxel)
        bool validCorner = false;
        if (clip.w > 0.0) {
            validCorner = true;
        }

        // after loop
        if (!validCorner) return;   // safe abort

        vec3 ndc = clip.xyz / clip.w;
        if (abs(ndc.x) <= 1.0 && abs(ndc.y) <= 1.0) anyCornerInNDC = true;

        vec2 pixel = (ndc.xy * 0.5 + vec2(0.5)) * vec2(u_viewport);
        minPix = min(minPix, pixel);
        maxPix = max(maxPix, pixel);
    }

    // Conservative off-screen check: if bbox entirely outside viewport, early out
    if (!anyCornerInNDC) {
        if (maxPix.x < 0.0 || maxPix.y < 0.0 || minPix.x >= float(u_viewport.x) || minPix.y >= float(u_viewport.y))
            return;
    }

    // Convert to integer pixel iteration range.
    // Use rounding to nearest and clamp to viewport extents.
    ivec2 vp = u_viewport;
    ivec2 pixMin = ivec2(floor(minPix + 0.5));
    ivec2 pixMax = ivec2(ceil (maxPix + 0.5));

    pixMin = clamp(pixMin, ivec2(0), vp - ivec2(1));
    pixMax = clamp(pixMax, ivec2(0), vp - ivec2(1));

    // Precompute ray origin in model space (camera origin transformed)
    vec3 rayOriginModel = (invMV * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

    // Iterate pixels (note: upper bound inclusive in pixMax -> use <=)
    for (int y = pixMin.y; y < pixMax.y; ++y) {
        for (int x = pixMin.x; x < pixMax.x; ++x) {
            // NDC of pixel center
            vec2 px = vec2(x, y) + vec2(0.5);
            vec2 ndc = (px / vec2(vp)) * 2.0 - vec2(1.0);

            // Unproject a direction through the near plane (-1 in clip/Eye z)
            vec4 clip = vec4(ndc, -1.0, 1.0);
            vec4 eye = invProj * clip;
            eye /= eye.w; // to get direction in view space

            // Ray direction in model space (transform direction vector)
            vec3 rayDirModel = normalize((invMV * vec4(eye.xyz, 0.0)).xyz);

            float tmin;
            float tmax;
            if (!intersectAABB(rayOriginModel, rayDirModel, boxMin, boxMax, tmin, tmax))
                continue;

            // prefer nearest positive intersection
            float tHit = (tmin > 0.0) ? tmin : tmax;
            if (tHit <= 0.0) continue;

            // intersection point not needed for color, we only write color
            ivec2 pxy = ivec2(x, y);
            // safety check (should be within bounds due to clamp above)
            if (pxy.x < 0 || pxy.y < 0 || pxy.x >= vp.x || pxy.y >= vp.y) continue;

            imageStore(imgColor, pxy, vec4(u_color, 1.0));
        }
    }
}
)";
