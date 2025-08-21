#include "cRenderer.h"



cRenderer::cRenderer(Entity* entity) :Component(entity), mModel(1.0f), mVertexBuffer(GL_ARRAY_BUFFER), mIndexBuffer(GL_ELEMENT_ARRAY_BUFFER)
{
    mTransform = GetMasterEntity()->GetTransformPointer();

    glGenVertexArrays(1, &mVertexArray);
    glBindVertexArray(mVertexArray);

    mVertexBuffer.Bind();
    mIndexBuffer.Bind();

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    InspectorVariables.push_back({"color", VEC4, glm::value_ptr(color), 0.01f});
    name = "Renderer";
}

cRenderer* cRenderer::SetRenderType(RENDER_TYPE aRenderType)
{
    mRenderType = aRenderType;
    return this;
}


void cRenderer::Render()
{
    mModel = glm::mat4(1.0f);
    mModel = glm::translate(mModel, mTransform->position);
    mModel = glm::rotate(mModel, mTransform->rotation.x, glm::vec3(1, 0, 0));
    mModel = glm::rotate(mModel, mTransform->rotation.y, glm::vec3(0, 1, 0));
    mModel = glm::rotate(mModel, mTransform->rotation.z, glm::vec3(0, 0, 1));
    mModel = glm::scale(mModel, mTransform->scale);


    glBindVertexArray(mVertexArray);
}
cRenderer* cRenderer::AddVertices(const std::vector<float>& aVertices)
{
    mVertexBuffer.Bind();
    mVertices = aVertices;
    mVertexBuffer.AttatchData(aVertices.size() * sizeof(float), mVertices.data(), GL_STATIC_DRAW);
    mVertexBuffer.Unbind();

    return this;
}

cRenderer* cRenderer::AddIndices(const std::vector<int>& aIndices)
{
    mIndexBuffer.Bind();
    mIndices = aIndices;
    mIndexBuffer.AttatchData(sizeof(int) * mIndices.size(), mIndices.data(), GL_STATIC_DRAW);
    mIndexBuffer.Unbind();

    return this;
}