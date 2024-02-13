#include "stogaEngine/engine.h"

engine::Instructions::Instructions(std::vector<engine::Waypoint>& i) {
    instructions = i;
}

engine::Waypoint& engine::Instructions::operator[](size_t index) {
    if (index >= instructions.size() || index < 0) throw InstructionOutOfRange();
    return instructions[index];
}

size_t engine::Instructions::size() {
    return instructions.size();
}

engine::Engine* engine::generateNewEngine(Instructions* x) {
    return new Engine(x);
}

engine::Engine::Engine(Instructions *i) : ins(i) {}

void engine::Engine::reconfigure(Instructions *x) { ins = x; }

void engine::Engine::executeAllCommands()
{
    
}
