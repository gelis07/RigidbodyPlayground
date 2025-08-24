#pragma once
#include "Utilities.h"
#include <imgui.h>
#include "Scene.h"
#include "Engine.h"





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