#include "stogaEngine/tos.h"
using namespace engine::presets;

engine::Waypoint::Waypoint(std::string& c, double p1, double p2, double p3)
    : command(c), param1(p1), param2(p2), param3(p3) {}

void IMUVectorOrientedTOS::initialize(AbstractDrivetrain* a, engine::SensorComponentList& l) {
    drivetrain = a;
    sensors = l;
}

std::array<double, 3> IMUVectorOrientedTOS::updateCoordinates() {
    std::array<double, 3> tor;
}
