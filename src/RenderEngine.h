#pragma once
#include <glad/glad.h>
#include "cRenderer.h"
#include "Shader.h"
#include "Utilities.h"
#include "Camera.h"

class RenderEngine
{
    public:
        void Init();
        void Update(const std::vector<cRenderer*>& renderers, const glm::mat4& aProjection, float time);
    private:
        static void GladErrorCallBack(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei lenght, const GLchar* message, const void *userParam);
        std::unordered_map<RENDER_TYPE, Shader> mShaders;
        Camera mCamera;

};