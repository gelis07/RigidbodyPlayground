#include "Scene.h"
#include <iostream>
#include <fstream>



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
                comp->InspectorVariables[v].Save(component);
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
        Entity& entity = entities[entities.size()-1];
        entity.SetTransform({position, rotation, scale});
        cRigidBody* objRigid1 = entity.AddComponent<cRigidBody>();
        cRenderer* objRend1 = entity.AddComponent<cRenderer>();
        // objRend1->color = glm::vec4(Utilities::RandomFloat(), Utilities::RandomFloat(), Utilities::RandomFloat(), 1.0f);
        objRend1->AddVertices(vertices);
        objRend1->AddIndices(indices);
        
        objRigid1->SetVertices(SquareVertices);
        objRigid1->Init();
        
        for(int c = 0; c < entity.mComponents.size(); c++)
        {
            Component* comp = entity.mComponents[c];
            json component = data[i][comp->name];
            for(int v = 0; v < comp->InspectorVariables.size(); v++)
            {
                comp->InspectorVariables[v].Load(component);
            }
        }

        for (auto& [name, value] : data[i].items()) 
        {
            if(name == objRend1->name || name == objRigid1->name || name == "transform")
                continue;

            json component = value;
            Component* comp = entity.AddComponent<LuaComponent>(name);
            for(int v = 0; v < comp->InspectorVariables.size(); v++)
            {
                comp->InspectorVariables[v].Load(component);
            }
            entity.mLuaComponents.push_back((LuaComponent*)entity.mComponents.back());
        }
        renderers.push_back(objRend1);
        rbs.push_back(objRigid1);
    }
    
}