#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glm.hpp>
#include <memory>
#include <vector>
#include <unordered_map>
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>
#include <unordered_set>
#define GLM_ENABLE_EXPERIMENTAL
#include <gtx/norm.hpp>
#define PI 3.141592

#define WIDTH 1280.0f
#define HEIGHT 920.0f

#define DEBUG 1

#ifdef DEBUG
#define Log(condition, msg) if(condition) std::cout << msg << '\n'
#else
#define Log(condition, msg)
#endif



enum VARIABLE_TYPE
{
    BOOL,
    FLOAT,
    VEC2,
    VEC3,
    VEC4
};


struct Transform
{
    glm::vec3 position;
    glm::vec3 rotation;
    glm::vec3 scale;
};


namespace Utilities
{


    inline float RoundToDecimal(float value, int place)
    {
        return glm::ceil(value * glm::pow(10, place)) / glm::pow(10, place);
    }
    inline glm::vec3 RoundToDecimal(glm::vec3 value, int place)
    {
        float x = glm::ceil(value.x * glm::pow(10, place)) / glm::pow(10, place);
        float y = glm::ceil(value.y * glm::pow(10, place)) / glm::pow(10, place);
        float z = glm::ceil(value.z * glm::pow(10, place)) / glm::pow(10, place);
        return glm::vec3(x,y,z);
    }
    // inline glm::vec3 Clamp(glm::vec3 value, glm::vec3 min, glm::vec3 max)
    inline glm::vec3 VectorSum(const std::vector<glm::vec3> vectors)
    {
        glm::vec3 sum(0);
        for(int i = 0; i < vectors.size(); i++)
            sum += vectors[i];
        
        return sum;
    }



    inline glm::vec3 ToWorldCoordinates(Transform trans, glm::vec3 point)
    {
        glm::mat4 model(1.0f);
        model = glm::translate(model, trans.position);
        model = glm::rotate(model, trans.rotation.x, glm::vec3(1, 0, 0));
        model = glm::rotate(model, trans.rotation.y, glm::vec3(0, 1, 0));
        model = glm::rotate(model, trans.rotation.z, glm::vec3(0, 0, 1));

        return glm::vec3(model * glm::vec4(point,1.0f));
    }
    inline bool AlmostEqual(float a, float b)
    {
        return glm::abs(a-b) < 0.00001f;
    }

    inline void print(const glm::vec3& vec)
    {
        std::cout << vec.x << ", " << vec.y << ", " << vec.z << '\n';
    }
    inline std::string LoadFileAsString(const std::string& aFileName)
    {
        std::string fLine;
        std::ostringstream fOutput;
        std::ifstream fFile(aFileName.c_str());
        while(std::getline(fFile, fLine))
        {
            fOutput << fLine<<'\n';
        }
        return fOutput.str();
    }
}