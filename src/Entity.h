#pragma once
#include "Utilities.h"


class Component;

class Entity
{
    public:
        void Update();

        const Transform& GetTransform() { return mTransform;}
        Transform* GetTransformPointer() {return &mTransform;}
        void SetTransform(const Transform& aTrans) {mTransform = aTrans;} 
        std::vector<Component*> mComponents;

        template <typename T> T* GetComponent()
        {
            for (auto& c : mComponents) {
                if (auto ptr = reinterpret_cast<T*>(c)) {
                    return ptr;
                }
            }
            return nullptr;
        }
        template<typename T, typename... Args> T* AddComponent(Args&&... args)
        {
            T* comp = new T(this, std::forward<Args>(args)...);
            mComponents.emplace_back(comp);
            return comp;
        }
    private:
        Transform mTransform;
        friend Component;
};
