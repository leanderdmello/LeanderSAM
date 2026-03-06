// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

class HUDElement
{
    public:

    ~HUDElement();

    void UpdateResources();

    void RenderElement(GLuint shaderId, GLuint textureId,
        const glm::mat4& view, const glm::mat4& projection);

    void RenderElement(GLuint shaderId, GLuint textureId,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::mat4& model);

    private:

    void ClearResources();
    
    void Render(GLuint shaderId, GLuint textureId,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::mat4& model);

    unsigned int _VBO, _VAO;

    protected:

    float _quad[16] = {
    // positions     // texCoords
    1.0,  1.0,           0.0, 0.0,
    1.0,  1.0,           1.0, 0.0,
    1.0,  1.0,           0.0, 1.0,
    1.0,  1.0,           1.0, 1.0,
    };
};

