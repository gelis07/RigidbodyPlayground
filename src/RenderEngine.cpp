#include "RenderEngine.h"
#include "Utilities.h"


void RenderEngine::GladErrorCallBack(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei lenght, const GLchar* message, const void *userParam)
{
    std::cout << message << '\n';
}

void RenderEngine::Init()
{
    mCamera.Init();
    Log(!gladLoadGL(), "Glad couldn't load gl");
    glDebugMessageCallback(GladErrorCallBack, NULL);

    Shader normalShader;
    Shader CircleShader;
    normalShader
    .AttachShader(GL_VERTEX_SHADER, "Basic.vs")
    ->AttachShader(GL_FRAGMENT_SHADER, "Basic.fs")
    ->Init();
    CircleShader
    .AttachShader(GL_VERTEX_SHADER, "Basic.vs")
    ->AttachShader(GL_FRAGMENT_SHADER, "CircleShader.fs")
    ->Init();
    
    mShaders[NORMAL] = normalShader;
    mShaders[CIRCLE] = CircleShader;
}
void RenderEngine::Update(const std::vector<cRenderer*>& renderers, const glm::mat4& aProjection, float time)
{
    glClearColor(0.5f, 0.2f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    for (int i = 0; i < renderers.size(); i++) 
    {
        cRenderer* obj = renderers[i];

        obj->Render(); // Just binds the needed vertex array for now.
        Shader& CurrentShader = mShaders.at(obj->GetRenderType()); // Get the needed shader.
        CurrentShader.Bind();
        CurrentShader.SetUniform("uColor", obj->color);
        glm::mat4 mvp = aProjection * mCamera.GetView() * obj->GetModel();
        CurrentShader.SetUniform("uMVP", mvp);


        if(obj->GetRenderType() == CIRCLE)
        {
            CurrentShader.SetUniform("uPos", glm::vec2(obj->GetTransform().position));
        }

        glDrawElements(GL_TRIANGLES, obj->IndicesCount, GL_UNSIGNED_INT, nullptr);
        glUseProgram(0);
        glBindVertexArray(0);
    }
}