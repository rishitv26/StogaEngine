#include "stogaEngine/tos.h"
using namespace engine::presets;

engine::Waypoint::Waypoint(std::string& c, double p1, double p2, double p3)
    : command(c), param1(p1), param2(p2), param3(p3) {}

IMUVectorOrientedTOS::IMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l) {
    drivetrain = a;
    sensors = l;
}

///////////////////////////////////////////// @TODO ================================================================== 
std::array<double, 3> IMUVectorOrientedTOS::updateCoordinates() {
    std::array<double, 3> tor;
}

std::array<double, 2> IMUVectorOrientedTOS::move(Waypoint& point) {

}

std::array<double, 2> IMUVectorOrientedTOS::power(Waypoint& point) {

}

std::array<double, 2> IMUVectorOrientedTOS::turn(Waypoint& point) {

}

bool IMUVectorOrientedTOS::is_running() {

}

engine::AbstractTemporaryOdomSystem* engine::generateNewIMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l) {
    return (engine::AbstractTemporaryOdomSystem*)(new engine::presets::IMUVectorOrientedTOS(a, l));
}
