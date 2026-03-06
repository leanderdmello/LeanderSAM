// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula


#pragma once

// Simple textured quad shaders to display color_texture to screen
const char* quad_vert_src = R"(#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
out vec2 vUV;

void main() {
    vUV = aUV;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

const char* quad_frag_src = R"(#version 330 core
in vec2 vUV;
out vec4 fragColor;
uniform sampler2D u_tex;

void main() {
    fragColor = texture(u_tex, vUV);
}
)";