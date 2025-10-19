#include "Editor.h"
#include <GLFW/glfw3.h>
#include <filesystem>
void Editor::Update()
{
    Dockspace();
    ImGui::Begin("Viewport");
    const float windowWidth = ImGui::GetContentRegionAvail().x;
    const float windowHeight = ImGui::GetContentRegionAvail().y;
    if(paused)
    {   
        ImGui::SetCursorPosX(windowWidth/2.0f);
        if(ImGui::Button(">"))
        {
            paused = !paused;
        }
    }else
    {
        ImGui::SetCursorPosX(windowWidth/2.0f);
        if(ImGui::Button("||"))
        {
            paused = !paused;
        }
    }

    RescaleFramebuffer(windowWidth, windowHeight);
    glViewport(0,0,windowWidth, windowHeight);
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ViewPos = glm::vec3(pos.x, pos.y + windowHeight, 0.0f);
    ImGui::GetWindowDrawList()->AddImage(
        reinterpret_cast<void*>(textureId), 
        ImVec2(pos.x, pos.y), 
        ImVec2(pos.x + windowWidth, pos.y + windowHeight), 
        ImVec2(0, 1), 
        ImVec2(1, 0)
    );
    scene.width = windowWidth;
    scene.height = windowHeight;

    ImGui::End();
    Hierachy();
    SceneManagement();
    DebugUI();

    ImGui::Begin("Inspector");
    if(SelectedEntity != -1)
    {
        ImGui::Text("Transform");
        Entity& selectedEntity = scene.entities[SelectedEntity]; 
        ImGui::DragFloat3("position", glm::value_ptr(selectedEntity.GetTransformPointer()->position));
        ImGui::DragFloat3("rotation", glm::value_ptr(selectedEntity.GetTransformPointer()->rotation), 0.01f);
        ImGui::DragFloat3("scale", glm::value_ptr(selectedEntity.GetTransformPointer()->scale));
        for (Component* component : selectedEntity.mComponents)
        {
            ImGui::Text("%s", component->name.c_str());
            for (InspectorVarData varData : component->InspectorVariables)
            {
                varData.Render();
            }
        }
        ImGui::Separator();
        if(ImGui::Button("Add Component"))
        {
            AddComponentWindow = true;
        }
    }
    ImGui::End();

    if(AddComponentWindow && SelectedEntity != -1)
    {
        Entity& selectedEntity = scene.entities[SelectedEntity]; 
        ImGui::Begin("Add Component");
        for(const auto& dirEntry : std::filesystem::recursive_directory_iterator(Stg::App::basePath + "Scripts/"))
        {
            if(ImGui::Button(dirEntry.path().string().c_str()))
            {
                selectedEntity.AddComponent<LuaComponent>(dirEntry.path().string());
                selectedEntity.mLuaComponents.push_back((LuaComponent*)selectedEntity.mComponents.back());
                AddComponentWindow = false;
            }
        }
        ImGui::Separator();
        if(ImGui::Button("Never mind"))
        {
            AddComponentWindow = false;
        }
        ImGui::End();
    }
}

void Editor::Timers()
{
    ImGui::Begin("Timers");

    for (const auto& [key, value] : Timings::GetTimings()) {
        ImGui::Text("%s: %.4f ms", key.c_str(), value);
    }

    ImGui::End();
}

void Editor::Hierachy()
{
    ImGui::Begin("Hiearachy");
    for (int i = 0; i < scene.entities.size(); i++) 
    {
        bool ChangedColor = false;
        if(SelectedEntity == i)
        {
            glm::vec3 panelColor = glm::vec3(0.17f, 0.18f, 0.19f);
            panelColor *= 1.8f;
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertFloat4ToU32(ImVec4(panelColor.r, panelColor.g, panelColor.b, 1.00f)));
            ChangedColor = true;
        }
        if(ImGui::Button(("Entity " + std::to_string(i)).c_str()))
        {
            SelectedEntity = i;
        }
        if(ChangedColor)
            ImGui::PopStyleColor();
    }

    if(ImGui::Button("+"))
    {
        scene.entities.push_back({});
        Entity& obj1 = scene.entities[scene.entities.size()-1];
        obj1.SetTransform({glm::vec3(0, 0, 0.0f), glm::vec3(0), glm::vec3(100.0f)});
        cRigidBody* objRigid1 = obj1.AddComponent<cRigidBody>();
        cRenderer* objRend1 = obj1.AddComponent<cRenderer>();
        objRend1->color = glm::vec4(utilities::RandomFloat(), utilities::RandomFloat(), utilities::RandomFloat(), 1.0f);
        objRend1->AddVertices(vertices);
        objRend1->AddIndices(indices);

        objRigid1->SetVertices(SquareVertices);
        objRigid1->Init();

        scene.renderers.push_back(objRend1);
        scene.rbs.push_back(objRigid1);
    }
    ImGui::End();
}


void Editor::DebugUI()
{
    ImGui::Begin("Settings");
    ImGui::Checkbox("Object Spawner", &Stg::Phs::EnableObjectSpawn);
    ImGui::InputInt("Physics Engine Iterations", &Stg::Phs::PhysicsEngineIterations);
    ImGui::InputInt("Gauss-Seidel Solver Iterations", &Stg::Phs::GaussSeidelIterations);
    ImGui::End();
}

void Editor::SceneManagement()
{
    ImGui::Begin("Scenes");
    ImGui::InputText("Save Scene Path", &SaveScenePath);
    if (ImGui::Button("save"))
    {
        scene.Save(Stg::App::basePath + "Scenes/" + SaveScenePath);
    }
    if(ImGui::Button("Load"))
    {
        LoadSceneWindow = true;
    }
    if(LoadSceneWindow)
    {
        ImGui::Begin("Load Scene");
        for(const auto& dirEntry : std::filesystem::recursive_directory_iterator(Stg::App::basePath + "Scenes/"))
        {
            if(ImGui::Button(dirEntry.path().string().c_str()))
            {
                std::string path = dirEntry.path().string();
                std::ifstream f(path);
                if(f.good())
                {
                    scene.Load(path);
                    SelectedEntity = -1;
                }
                else
                    fmt::println("Cant open file");

                LoadSceneWindow = false;
            }
        }
        ImGui::Separator();
        if(ImGui::Button("Never mind"))
        {
            LoadSceneWindow = false;
        }
        ImGui::End();
    }
    if(ImGui::Button("Delete"))
    {
        scene.Delete();
        scene.entities.reserve(100);
        SelectedEntity = -1;
    }
    ImGui::End();
}
void Editor::Dockspace()
{
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking |
                                    ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoCollapse |
                                    ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoMove |
                                    ImGuiWindowFlags_NoBringToFrontOnFocus |
                                    ImGuiWindowFlags_NoNavFocus;

    // Important: only PassthruCentralNode lets the clear color show through
    ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode; 

    // Push ONLY for the DockSpace host
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

    ImGui::Begin("DockSpace", nullptr, window_flags);
    ImGui::PopStyleVar(3); // pop only here

    // Dockspace node
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

    ImGui::End();
}

void Editor::Init()
{

    scene.entities.reserve(100);
    scene.entities.resize(2);
    Entity& obj1 = scene.entities[1];
    Entity& obj2 = scene.entities[0];
    obj1.SetTransform({glm::vec3(WIDTH/2.0f, 1000, 0.0f), glm::vec3(0), glm::vec3(100.0f)});
    cRigidBody* objRigid1 = obj1.AddComponent<cRigidBody>();
    cRenderer* objRend1 = obj1.AddComponent<cRenderer>();
    objRend1->AddVertices(vertices);
    objRend1->AddIndices(indices);

    objRigid1->SetVertices(SquareVertices);
    objRigid1->Init();

    obj2.SetTransform({glm::vec3(500, 100, 0.0f), glm::vec3(0), glm::vec3(230.0f, 10.0f, 0.0f)});
    cRigidBody* objRigid2 = obj2.AddComponent<cRigidBody>();
    cRenderer* objRend2 = obj2.AddComponent<cRenderer>();
    objRend2->AddVertices(vertices);
    objRend2->AddIndices(indices);

    objRigid2->SetVertices(SquareVertices);
    objRigid2->Init();
    objRigid2->SetStatic(true);
    

    scene.renderers.push_back(objRend1);
    scene.renderers.push_back(objRend2);
    
    scene.rbs.push_back(objRigid1);
    scene.rbs.push_back(objRigid2);



    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureId, 0);

    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, WIDTH, HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!\n";
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    SelectedEntity = 0;
}

void Editor::RescaleFramebuffer(float width, float height)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glBindTexture(GL_TEXTURE_2D, textureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureId, 0);

	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
}