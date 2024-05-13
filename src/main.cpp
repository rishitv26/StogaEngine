/*
*   ========================================= Code Entry Point =====================================
*   
*   Entry point of the code. Uses movements defined in Robot in order to move and follow routines.
*   This file is not to be messed with, unless you know what your doing.
*
*   - Rishit Varshney
*/


#include "main.h"
#include "settings.h"
#include "robot.h"

Robot robot;

void initialize() {
	pros::lcd::initialize();
	robot.setup();
}

void disabled() {
	robot.components.haltAll();
}

void competition_initialize() {}

// job of interpreter...
void autonomous() {
	
}

void opcontrol() {
	robot.driveControl();	
}