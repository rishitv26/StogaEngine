#include "stogaEngine/robot.h"

void engine::Bot::initInstances() {
    master.initialize();
}

engine::Bot::~Bot() {
    delete odom;
    delete drivetrain;
}

void engine::Bot::defaultDriveBehavior() {
    components.bindAll(master);
    sensors.updateAll();
    pros::delay(2);
}

void engine::Bot::driveControl() {
    defaultDriveBehavior();
}
