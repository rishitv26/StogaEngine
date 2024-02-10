#ifndef ENGINE_SE_H
#define ENGINE_SE_H

#include "tos.h"

namespace engine {
class Instructions {
private:
    std::vector<engine::Waypoint> instructions;
public:
    explicit Instructions(std::vector<engine::Waypoint>& i);
    engine::Waypoint& operator[](size_t index);

    size_t size();
};

class Engine {
private:
    Instructions* ins;
public:
    explicit Engine() {};
    explicit Engine(Instructions* x);
};

Engine* generateNewEngine(Instructions* x);
};

#endif ENGINE_SE_H