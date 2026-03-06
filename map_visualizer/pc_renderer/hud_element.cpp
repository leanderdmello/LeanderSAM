// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "hud_element.h"

#include <glm/gtc/type_ptr.hpp>

HUDElement::~HUDElement() {
    ClearResources();
}

void HUDElement::ClearResources() {
    if(_VAO != 0)
        glDeleteVertexArrays(1, &_VAO);
    if(_VBO != 0)
        glDeleteBuffers(1, &_VBO);
}

void HUDElement::UpdateResources() {
    ClearResources();
    glGenVertexArrays(1, &_VAO);
    glGenBuffers(1, &_VBO);
    glBindVertexArray(_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, _VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_quad), _quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

void HUDElement::RenderElement(GLuint shaderId, GLuint textureId,
    const glm::mat4& view, const glm::mat4& projection) {
    Render(shaderId, textureId, view, projection, glm::mat4(1.f));
}

void HUDElement::RenderElement(GLuint shaderId, GLuint textureId,
    const glm::mat4& view, const glm::mat4& projection,
    const glm::mat4& model) {
    Render(shaderId, textureId, view, projection, model);
}

void HUDElement::Render(GLuint shaderId, GLuint textureId,
    const glm::mat4& view, const glm::mat4& projection,
    const glm::mat4& model) {
    glUseProgram(shaderId);

    glBindTexture(GL_TEXTURE_2D, textureId);

    glUniform1i(glGetUniformLocation(shaderId, "hudTexture"), 0);

    GLint loc_view = glGetUniformLocation(shaderId, "view");
    glUniformMatrix4fv(loc_view, 1, GL_FALSE, glm::value_ptr(view));

    GLint loc_proj = glGetUniformLocation(shaderId, "projection");
    glUniformMatrix4fv(loc_proj, 1, GL_FALSE, glm::value_ptr(projection));

    GLint loc_model = glGetUniformLocation(shaderId, "model");
    glUniformMatrix4fv(loc_model, 1, GL_FALSE, glm::value_ptr(model));

    glBindVertexArray(_VAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glActiveTexture(GL_TEXTURE0);
}