#include "Engine.h"
#include "imgui.h"
#include "App.h"

void Engine::Init()
{
    mRenderEngine.Init();
    mPhysicsEngine.Init();
    mProjection = glm::ortho(0.0f, WIDTH, 0.0f, HEIGHT, -1.0f, 1.0f);
}
std::unordered_map<int, bool> prevKeys;
bool GetKeyDown(GLFWwindow* window, int key) {
    bool isPressed = glfwGetKey(window, key) == GLFW_PRESS;
    bool wasPressed = prevKeys[key];
    prevKeys[key] = isPressed; // store for next frame
    return isPressed && !wasPressed;
}
std::unordered_map<int, bool> mouseKeyPrev;
bool GetClickDown(GLFWwindow* window, int key) {
    bool isPressed = glfwGetMouseButton(window, key) == GLFW_PRESS;
    bool wasPressed = mouseKeyPrev[key];
    mouseKeyPrev[key] = isPressed; // store for next frame
    return isPressed && !wasPressed;
}

void Engine::Run(Scene* scene,bool paused, float time)
{
    ImVec2 p = ImGui::GetWindowPos();
    glm::vec3 pos(p.x, p.y, 0.0f);
    mProjection = glm::ortho(0.0f, scene->width, 0.0f, scene->height, -1.0f, 1.0f);

    float currentTime = glfwGetTime();
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;
    if(paused)
    {
        if(GetKeyDown(glfwGetCurrentContext(), GLFW_KEY_RIGHT))
        {
            for (int iter = 0; iter < Stg::Phs::PhysicsEngineIterations; iter++) 
            {
                mPhysicsEngine.Update(scene->rbs, deltaTime / Stg::Phs::PhysicsEngineIterations);
            }
        }
    }else
    {
        Timings::StartTimer("Whole Engine");
        if (Stg::Phs::EnableObjectSpawn && GetClickDown(glfwGetCurrentContext(), GLFW_MOUSE_BUTTON_LEFT))
        {
            double xpos, ypos;
            glfwGetCursorPos(glfwGetCurrentContext(), &xpos, &ypos);
            glm::vec3 CursorPos(xpos, ypos, 0.0f);
            glm::vec3 final((CursorPos-ViewPos).x, -(CursorPos-ViewPos).y , 0.0f);
            if(final.x > 0 && final.y > 0)
            {
                scene->entities.push_back({});
                Entity& obj1 = scene->entities[scene->entities.size()-1];
                obj1.SetTransform({final, glm::vec3(0), glm::vec3(50.0f)});
                cRigidBody* objRigid1 = obj1.AddComponent<cRigidBody>();
                cRenderer* objRend1 = obj1.AddComponent<cRenderer>();
                objRend1->color = glm::vec4(utilities::RandomFloat(), utilities::RandomFloat(), utilities::RandomFloat(), 1.0f);
                objRend1->AddVertices(vertices);
                objRend1->AddIndices(indices);

                objRigid1->SetVertices(SquareVertices);
                objRigid1->Init();

                scene->renderers.push_back(objRend1);
                scene->rbs.push_back(objRigid1);
            }
        }

        for (int iter = 0; iter < Stg::Phs::PhysicsEngineIterations; iter++) 
        {
            mPhysicsEngine.Update(scene->rbs, deltaTime / Stg::Phs::PhysicsEngineIterations);
        }
        Timings::EndTimer();
    }
    if(GetKeyDown(glfwGetCurrentContext(), GLFW_KEY_SPACE))
        paused = !paused;
    for (int i = 0; i < scene->entities.size(); i++)
    {
        Entity& entity = scene->entities[i];
        entity.Update();
    }
    mRenderEngine.Update(scene->renderers, mProjection, time);
}