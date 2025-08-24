#include "Shader.h"


Shader::Shader()
{
    mId = glCreateProgram();
}

Shader* Shader::AttachShader(int ShaderType, std::string aFileName)
{
    GLuint shader = glCreateShader(ShaderType);
    std::string shaderSourceStr = Utilities::LoadFileAsString(aFileName);
    const char* shaderSource = shaderSourceStr.c_str();
    glShaderSource(shader, 1, &shaderSource, NULL);
    glCompileShader(shader);
    {
        int  success;
        char infoLog[512];
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if(!success)
        {
            glGetShaderInfoLog(shader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
        }
    }
    glAttachShader(mId, shader);
    return this;
}

void Shader::SetUniform(const std::string& name, const glm::vec4& data)
{
    glUniform4f(glGetUniformLocation(mId, name.c_str()), data.x, data.y, data.z, data.w);
}
void Shader::SetUniform(const std::string& name, const float& data)
{
    glUniform1f(glGetUniformLocation(mId, name.c_str()), data);
}
void Shader::SetUniform(const std::string& name, const glm::vec2& data)
{
    glUniform2f(glGetUniformLocation(mId, name.c_str()), data.x, data.y);
}
void Shader::SetUniform(const std::string& name, const glm::vec3& data)
{
    glUniform3f(glGetUniformLocation(mId, name.c_str()), data.x, data.y, data.z);
}
void Shader::SetUniform(const std::string& name, const glm::mat4& data)
{
    glUniformMatrix4fv(glGetUniformLocation(mId, name.c_str()), 1, GL_FALSE, glm::value_ptr(data));
}

void Shader::Bind()
{
    glUseProgram(mId);
}

void Shader::Unbind()
{
    glUseProgram(0);
}

void Shader::Init()
{
    glLinkProgram(mId);
}