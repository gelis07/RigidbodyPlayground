#include "Entity.h"
#include "imgui.h"
#include "Component.h"


void Entity::Update()
{

    for (int i = 0; i < mLuaComponents.size(); i++)
    {
        mLuaComponents[i]->Update();
    }
}