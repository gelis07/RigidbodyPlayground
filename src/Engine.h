#pragma once
#include "Utilities.h"
#include "PhysicsEngine.h"
#include "RenderEngine.h"
#include "Scene.h"


class Engine
{
    public:
        void Init();
        void Run(Scene* scene, float time);
        RenderEngine mRenderEngine;
        PhysicsEngine mPhysicsEngine;
    private:
        Entity CircleIndicator;
        glm::mat4 mProjection = glm::mat4(1.0f);
        bool paused = true;
        float lastTime = 0.0f;
        float accumalator = 0.0f;
}; 