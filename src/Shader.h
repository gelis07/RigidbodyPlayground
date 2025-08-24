#pragma once
#include <glad/glad.h>
#include "Utilities.h"


/*
    *Builder Design pattern

    Fist implement everything from the below function and then call the init() function
*/
class Shader
{
    public:
        Shader();
        Shader* AttachShader(int ShaderType, std::string aFileName);
        void Init();
        void Bind();
        void Unbind();
        void SetUniform(const std::string& name, const glm::vec4& data);
        void SetUniform(const std::string& name, const glm::vec3& data);
        void SetUniform(const std::string& name, const glm::vec2& data);
        void SetUniform(const std::string& name, const glm::mat4& data);
        void SetUniform(const std::string& name, const float& data);
    private:
        GLuint mId;
};