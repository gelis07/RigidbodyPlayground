#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include "Editor.h"
#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_glfw.h"
//App handles windows too
class App
{
    public:
        void Init();
        void Update();
    private:
        static void GLFWErrorCallBack(int error, const char* description);
        GLFWwindow* mWindow;
        Engine mEngine;
        Editor mEditor;
};