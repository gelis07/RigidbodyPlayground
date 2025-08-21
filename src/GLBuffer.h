#pragma once
#include <glad/glad.h>

/*
    *Builder Design pattern

    Fist implement everything from the below function and then call the init() function
*/

class GLBuffer
{
    public:
        GLBuffer(GLuint aBufferType);
        void Bind();
        GLBuffer* AttatchData(size_t size, void* data, GLuint usage);
        void Unbind();
    private:
        GLuint mBufferType;
        GLuint mId;
};  