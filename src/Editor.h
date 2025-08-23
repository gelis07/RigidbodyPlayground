#pragma once
#include "Utilities.h"
#include <imgui.h>
#include "Scene.h"
#include "Engine.h"


inline std::vector<float> vertices = 
{
    -0.5f,  0.5f, 0.0f,
    -0.5f, -0.5f, 0.0f,
     0.5f, -0.5f, 0.0f,
     0.5f,  0.5f, 0.0f 
};
inline std::vector<glm::vec3> SquareVertices = 
{
    glm::vec3(-0.5f,  0.5f, 0.0f),
    glm::vec3(-0.5f, -0.5f, 0.0f),
    glm::vec3( 0.5f, -0.5f, 0.0f),
    glm::vec3( 0.5f,  0.5f, 0.0f) 
};


inline std::vector<int> indices = 
{
    0, 1, 3,
    3, 1, 2
};


class Editor
{
    public:
        void Init();
        void Update();
        GLuint fbo;
        Scene scene;
        Scene SavedScene;
        bool paused = true;
    private:
        void Dockspace();
        void RenderVariable(InspectorVarData& data);
        void RescaleFramebuffer(float width, float height);
        void Hierachy();
        GLuint rbo;
        GLuint textureId;
        int SelectedEntity;
};