#include "Engine.h"
#include "imgui.h"
#include "App.h"
#define PHYSICS_TIMESTEP 0.02f
#define PHYSICS_ITERATIONS 10.0f
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

void Engine::Run(Scene* scene,bool paused, float time)
{

    mProjection = glm::ortho(0.0f, scene->width, 0.0f, scene->height, -1.0f, 1.0f);

    float currentTime = glfwGetTime();
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    if(paused)
    {
        if(GetKeyDown(glfwGetCurrentContext(), GLFW_KEY_RIGHT))
            mPhysicsEngine.Update(scene->rbs, PHYSICS_TIMESTEP);
    }else
    {
        accumalator += deltaTime;
        while (accumalator >= 0.02f)
        {
            for (int iter = 0; iter < PHYSICS_ITERATIONS; iter++) 
            {
                mPhysicsEngine.Update(scene->rbs, PHYSICS_TIMESTEP / PHYSICS_ITERATIONS);
            }
            accumalator -= 0.02f;
        }
    }
    if(GetKeyDown(glfwGetCurrentContext(), GLFW_KEY_SPACE))
        paused = !paused;
    // for (int i = 0; i < scene->entities.size(); i++)
    // {
    //     Entity& entity = scene->entities[i];
    //     entity.Update();
    // }
    mRenderEngine.Update(scene->renderers, mProjection, time);
}