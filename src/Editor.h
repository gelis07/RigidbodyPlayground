#pragma once
#include "Utilities.h"
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include "Scene.h"
#include "Engine.h"
#include "Timings.h"
#include "Settings.h"



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
        glm::vec3 ViewPos;
    private:
        void Dockspace();
        void RescaleFramebuffer(float width, float height);
        void Hierachy();
        void DebugUI();

        std::string SaveScenePath = "test.json";
        void SceneManagement();

        bool AddComponentWindow = false;
        bool LoadSceneWindow = false;
        GLuint rbo;
        GLuint textureId;
        int SelectedEntity;

};