#pragma once
#include "Utilities.h"
#include "GLBuffer.h"
#include "Component.h"
#include "Entity.h"
enum RENDER_TYPE
{
    NORMAL,
    CIRCLE
};

class cRenderer : public Component
{
    public:
        cRenderer(Entity* entity);
        void Render();
        cRenderer* AddVertices(const std::vector<float>& aVertices);
        cRenderer* AddIndices(const std::vector<int>& aIndices);
        cRenderer* SetRenderType(RENDER_TYPE aRenderType);
        const Transform& GetTransform() {return *mTransform;}
        void SetTransform(const Transform& aTrans);
        RENDER_TYPE GetRenderType() {return mRenderType;}
        const glm::mat4& GetModel() {return mModel;}
        int IndicesCount;
        glm::vec4 color = glm::vec4(1.0f);
    private:
        Transform* mTransform;
        std::vector<float> mVertices;
        std::vector<int> mIndices;
        GLBuffer mVertexBuffer;
        RENDER_TYPE mRenderType = NORMAL;
        GLBuffer mIndexBuffer;
        GLuint mVertexArray;
        glm::mat4 mModel;
};