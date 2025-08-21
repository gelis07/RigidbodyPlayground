#include "GLBuffer.h"



GLBuffer::GLBuffer(GLuint aBufferType) : mBufferType(aBufferType)
{
    glGenBuffers(1, &mId);
}

void GLBuffer::Bind()
{
    glBindBuffer(mBufferType,mId);
}


GLBuffer* GLBuffer::AttatchData(size_t size, void* data, GLuint usage)
{
    glBufferData(mBufferType, size, data, usage);
    return this;
}

void GLBuffer::Unbind()
{
    glBindBuffer(mBufferType, 0);
}