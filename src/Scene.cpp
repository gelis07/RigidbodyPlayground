#include "Scene.h"
#include <iostream>
#include <fstream>


void Scene::SaveVariable(json& data, InspectorVarData& var)
{
   switch (var.type)
    {
        case BOOL:
        {
            data[var.name] = *(bool*)(var.data);
            break;
        }
        case FLOAT:
        {
            data[var.name] = *(float*)(var.data);
            break;
        }
        case VEC2:
        {
            data[var.name]["x"] = (*(glm::vec2*)(var.data)).x;
            data[var.name]["y"] = (*(glm::vec2*)(var.data)).y;
            break;
        }
        case VEC3:
        {
            data[var.name]["x"] = (*(glm::vec3*)(var.data)).x;
            data[var.name]["y"] = (*(glm::vec3*)(var.data)).y;
            data[var.name]["z"] = (*(glm::vec3*)(var.data)).z;
            break;
        }
        case VEC4:
        {
            data[var.name]["x"] = (*(glm::vec4*)(var.data)).x;
            data[var.name]["y"] = (*(glm::vec4*)(var.data)).y;
            data[var.name]["z"] = (*(glm::vec4*)(var.data)).z;
            data[var.name]["w"] = (*(glm::vec4*)(var.data)).w;
            break;
        }
    }
}
void Scene::LoadVariable(json& data, InspectorVarData& var)
{
   switch (var.type)
    {
        case BOOL:
        {
            *(bool*)(var.data) = data[var.name];
            break;
        }
        case FLOAT:
        {
            *(float*)(var.data) = data[var.name];
            break;
        }
        case VEC2:
        {
            (*(glm::vec2*)(var.data)).x = data[var.name]["x"];
            (*(glm::vec2*)(var.data)).y = data[var.name]["y"];
            break;
        }
        case VEC3:
        {
            (*(glm::vec3*)(var.data)).x = data[var.name]["x"];
            (*(glm::vec3*)(var.data)).y = data[var.name]["y"];
            (*(glm::vec3*)(var.data)).z = data[var.name]["z"];
            break;
        }
        case VEC4:
        {
            (*(glm::vec4*)(var.data)).x = data[var.name]["x"];
            (*(glm::vec4*)(var.data)).y = data[var.name]["y"];
            (*(glm::vec4*)(var.data)).z = data[var.name]["z"];
            (*(glm::vec4*)(var.data)).w = data[var.name]["w"];
            break;
        }
    }
}

void Scene::Save(const std::string& path)
{
    json data;
    for (int i = 0; i < entities.size(); i++)
    {
        json object;
        json transform;

        json position;
        position["x"] = entities[i].GetTransform().position.x;
        position["y"] = entities[i].GetTransform().position.y;
        position["z"] = entities[i].GetTransform().position.z;
        json rotation;
        rotation["x"] = entities[i].GetTransform().rotation.x;
        rotation["y"] = entities[i].GetTransform().rotation.y;
        rotation["z"] = entities[i].GetTransform().rotation.z;
        json scale;
        scale["x"] = entities[i].GetTransform().scale.x;
        scale["y"] = entities[i].GetTransform().scale.y;
        scale["z"] = entities[i].GetTransform().scale.z;

        transform["position"] = position;
        transform["rotation"] = rotation;
        transform["scale"] = scale;
        object["transform"] = transform;


        for (int j = 0; j < entities[i].mComponents.size(); j++)
        {
            json component;
            Component* comp = entities[i].mComponents[j];
            for(int v = 0; v < comp->InspectorVariables.size(); v++)
            {
                SaveVariable(component, comp->InspectorVariables[v]);
            }
            object[comp->name] = component;
        }

        data[i] = object;
    }


    std::ofstream file(path.c_str());
    file << data.dump(2);
    file.close();
}


void Scene::Delete()
{
    for (int i = 0; i < rbs.size(); i++)
    {
        delete rbs[i];
    }
    for (int i = 0; i < renderers.size(); i++)
    {
        delete renderers[i];
    }

    entities.clear();
    rbs.clear();
    renderers.clear();
}

void Scene::Load(const std::string& path)
{
    Delete();
    entities.reserve(100);
    std::ifstream file(path.c_str());
    json data = json::parse(file);


    for (int i = 0; i < data.size(); i++)
    {

        glm::vec3 position;
        position.x = data[i]["transform"]["position"]["x"];
        position.y = data[i]["transform"]["position"]["y"];
        position.z = data[i]["transform"]["position"]["z"];
        glm::vec3 rotation;
        rotation.x = data[i]["transform"]["rotation"]["x"];
        rotation.y = data[i]["transform"]["rotation"]["y"];
        rotation.z = data[i]["transform"]["rotation"]["z"];
        glm::vec3 scale;
        scale.x = data[i]["transform"]["scale"]["x"];
        scale.y = data[i]["transform"]["scale"]["y"];
        scale.z = data[i]["transform"]["scale"]["z"];

        entities.push_back({});
        Entity& obj1 = entities[entities.size()-1];
        obj1.SetTransform({position, rotation, scale});
        cRigidBody* objRigid1 = obj1.AddComponent<cRigidBody>();
        cRenderer* objRend1 = obj1.AddComponent<cRenderer>();
        // objRend1->color = glm::vec4(Utilities::RandomFloat(), Utilities::RandomFloat(), Utilities::RandomFloat(), 1.0f);
        objRend1->AddVertices(vertices);
        objRend1->AddIndices(indices);
        
        objRigid1->SetVertices(SquareVertices);
        objRigid1->Init();
        
        for(int c = 0; c < obj1.mComponents.size(); c++)
        {
            Component* comp = obj1.mComponents[c];
            json component = data[i][comp->name];
            for(int v = 0; v < comp->InspectorVariables.size(); v++)
            {
                LoadVariable(component, comp->InspectorVariables[v]);
            }
        }

        renderers.push_back(objRend1);
        rbs.push_back(objRigid1);
    }
    
}