#include "stogaEngine/robot.h"

void engine::Bot::initInstances() {
    master.initialize();
}

engine::Bot::~Bot() {
    delete odom;
    delete drivetrain;
}

void engine::Bot::defaultDriveBehavior(pros::controller_analog_e_t move, pros::controller_analog_e_t turn, int drift) {
    int m = master.controller->get_analog(move);
    int t = master.controller->get_analog(turn);
    if (abs(m) < drift) m = 0;
    if (abs(t) < drift) t = 0;

    drivetrain->set_raw_analog(m - t, m + t);    
    components.bindAll(master);
    sensors.updateAll();
    pros::delay(2);
}

void engine::Bot::driveControl() {
    defaultDriveBehavior();
}
