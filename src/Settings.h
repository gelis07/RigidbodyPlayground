#pragma once
#include "string"


namespace Stg 
{
    namespace App {
        inline std::string basePath;
    }

    namespace Phs
    {
        inline bool EnableObjectSpawn = true;
        inline int PhysicsEngineIterations = 1;
        inline int GaussSeidelIterations = 15;
    }
}