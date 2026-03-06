// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
#include <rclcpp/rclcpp.hpp>

static GLuint compileShader(const rclcpp::Logger &logger, GLenum type, const char* src) {
    GLint status;
    GLuint shader_id = glCreateShader(type);
    
    glShaderSource(shader_id, 1, &src, nullptr);
    glCompileShader(shader_id);
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &status);
    
    if (!status) {
        GLint len = 0; glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &len);
        std::string err(len, ' ');
        glGetShaderInfoLog(shader_id, len, nullptr, err.data());
        RCLCPP_ERROR(logger, "Shader compile error: %s", err.c_str());
        exit(1);
    }
    return shader_id;
}

static GLuint linkProgramFromShaders(const rclcpp::Logger &logger, const std::vector<GLuint>& shaders) {
    GLint status;
    GLuint shader_program = glCreateProgram();
    for (const auto& shader : shaders)
        glAttachShader(shader_program, shader);
    
    glLinkProgram(shader_program); 
    glGetProgramiv(shader_program, GL_LINK_STATUS, &status);
    
    if (!status) {
        GLint len = 0; glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &len);
        std::string err(len, ' ');
        glGetProgramInfoLog(shader_program, len, nullptr, err.data());
        RCLCPP_ERROR(logger, "Program link error: %s", err.c_str());
        exit(1);
    }
    
    for (auto& shader : shaders) {
        glDetachShader(shader_program, shader);
        glDeleteShader(shader);
    }
    return shader_program;
}
