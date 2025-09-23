#pragma once
#include "Utilities.h"
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include "Scene.h"
#include "Engine.h"
#include "Timings.h"




class Editor
{
    public:
        void Init();
        void Update();
        void Timers();
        GLuint fbo;
        Scene scene;
        Scene SavedScene;
        bool paused = true;
    private:
        void Dockspace();
        void RenderVariable(InspectorVarData& data);
        void RescaleFramebuffer(float width, float height);
        void Hierachy();

        std::string LoadScenePath;
        std::string SaveScenePath = "test.json";
        void SceneManagement();


        GLuint rbo;
        GLuint textureId;
        int SelectedEntity;

};