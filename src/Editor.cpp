#include "Editor.h"

void Editor::Update()
{
    Dockspace();
    ImGui::Begin("Viewport");

    const float windowWidth = ImGui::GetContentRegionAvail().x;
    const float windowHeight = ImGui::GetContentRegionAvail().y;

    RescaleFramebuffer(windowWidth, windowHeight);
    glViewport(0,0,windowWidth, windowHeight);
    ImVec2 pos = ImGui::GetCursorScreenPos();

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

    ImGui::Begin("Inspector");
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
            RenderVariable(varData);
        }
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
            ImGui::PushStyleColor(ImGuiCol_Button, ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.0f, 0.0f, 1.0f)));
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
        objRend1->AddVertices(vertices);
        objRend1->AddIndices(indices);

        objRigid1->SetVertices(SquareVertices);
        objRigid1->Init();

        scene.renderers.push_back(objRend1);
        scene.rbs.push_back(objRigid1);
    }
    ImGui::End();
}

void Editor::RenderVariable(InspectorVarData& data)
{
    switch (data.type)
    {
        case BOOL:
        {
            ImGui::Checkbox(data.name.c_str(), reinterpret_cast<bool*>(data.data));
            break;
        }
        case FLOAT:
        {
            ImGui::DragFloat(data.name.c_str(), reinterpret_cast<float*>(data.data), data.speed);
            break;
        }
        case VEC2:
        {
            ImGui::DragFloat2(data.name.c_str(), reinterpret_cast<float*>(data.data), data.speed);
            break;
        }
        case VEC3:
        {
            ImGui::DragFloat3(data.name.c_str(), reinterpret_cast<float*>(data.data), data.speed);
            break;
        }
        case VEC4:
        {
            ImGui::DragFloat4(data.name.c_str(), reinterpret_cast<float*>(data.data), data.speed);
            break;
        }
    }
}


void Editor::Dockspace()
{
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    // Window flags for the dockspace host
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | 
                                    ImGuiWindowFlags_NoTitleBar | 
                                    ImGuiWindowFlags_NoCollapse | 
                                    ImGuiWindowFlags_NoResize | 
                                    ImGuiWindowFlags_NoMove | 
                                    ImGuiWindowFlags_NoBringToFrontOnFocus | 
                                    ImGuiWindowFlags_NoNavFocus | 
                                    ImGuiWindowFlags_NoBackground;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::Begin("DockSpace Demo", nullptr, window_flags);
    ImGui::PopStyleVar(2);

    // Dockspace
    ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
    ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode; 
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