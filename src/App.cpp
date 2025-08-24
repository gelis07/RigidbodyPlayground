#include "App.h"
#include "Utilities.h"

void App::GLFWErrorCallBack(int error, const char* description)
{
    std::cout << description << '\n';
}

void App::Init()
{
    Log(!glfwInit(), "Couldn't Initialize glfw");
    glfwSetErrorCallback(GLFWErrorCallBack);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
    glfwWindowHint(GLFW_MAXIMIZED, true);
    mWindow = glfwCreateWindow(WIDTH, HEIGHT, "Physics Engine 2.0", NULL, NULL);
    Log(!mWindow, "Couldn't create window or OpenGL Context");
    glfwMakeContextCurrent(mWindow);
    glfwSwapInterval(true);

    mEngine.Init();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    ImGui_ImplGlfw_InitForOpenGL(mWindow,true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();
    io.ConfigFlags = ImGuiConfigFlags_DockingEnable;
    mEditor.Init();
}


void App::Update()
{
    while(!glfwWindowShouldClose(mWindow))
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        glClearColor(0.0f, 0.2f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);



        mEditor.Update();
            
        glBindFramebuffer(GL_FRAMEBUFFER, mEditor.fbo);
        mEngine.Run(&mEditor.scene, mEditor.paused , glfwGetTime());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }
}