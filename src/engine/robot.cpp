#include "stogaEngine/robot.h"

void engine::Bot::init() {
    master.initialize();
    drivetrain.initialize();
    odom.initialize(drivetrain);
}
