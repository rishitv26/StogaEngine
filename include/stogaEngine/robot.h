/**
 * @file robot.h
 * @author Rishit Varshney
 * @brief This file contains the parent class of the important Robot class.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ROBOT_SE_H
#define ROBOT_SE_H

namespace engine {
class Robot {
protected:
	/*
	* The Chassis Object:
	* Represents the Chassis of the robot as a set of motors to control.
	* Also Interfaces Engine with motors in order to move robot during autons.
	**/
	engine::Chassis chassis;

	/*
	* The Utilities Object:
	* Represents all other components of the robot (ex. lift, intake, etc.)
	* Also Interfaces Engine with these components in order to move robot during autons.
	**/
	engine::Utilities utilities;
public:
};
};

#endif // ROBOT_SE_H
