#define GLFW_INCLUDE_NONE
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <sstream>
#include "App.h"



int main()
{
    App app;
    app.Init();
    app.Update();
}