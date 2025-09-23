#pragma once
#include "Utilities.h"
#include <stack>
#include <chrono>

using timevar = std::chrono::time_point<std::chrono::high_resolution_clock>;




class Timings
{
    private:
        struct TempTiming
        {
            std::string name;
            timevar time;
        };
    public:
        inline static void StartTimer(const std::string& name)
        {
            timevar time = std::chrono::high_resolution_clock::now();
            timingStack.push({name, time});
        }
        inline static void EndTimer()
        {
            if(timingStack.size() == 0)
                throw std::invalid_argument("There's no timer to end");
            timevar time = std::chrono::high_resolution_clock::now();

            double duration = std::chrono::duration<double, std::milli>(time - timingStack.top().time).count();
            timings[timingStack.top().name] = duration;
            timingStack.pop();
        }

        inline static const std::unordered_map<std::string, double>& GetTimings() {return timings;}
        inline static void ClearTimers() { timings.clear(); }

    private:
        inline static std::unordered_map<std::string, double> timings;
        inline static std::stack<TempTiming> timingStack;


};