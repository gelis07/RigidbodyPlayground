#pragma once
#include "Utilities.h"

class Camera
{
    public:
        void Init();
        void Update();
        const glm::mat4& GetView() {return mView;}
    private:
        glm::vec3 mPosition;
        glm::mat4 mView = glm::mat4(1.0f);
};