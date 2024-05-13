/**
 * @file chassis.h
 * @author Rishit Varshney
 * @brief This file contains the chassis class for movement and its subordinate helper classes and methods.
 * @version 0.1
 * @date 2024-05-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef CHASSIS_SE_H
#define CHASSIS_SE_H

#include "pid.h"

namespace engine {
	class Chassis {
	private:
	public:
		Chassis();
		/*
		 * Sets the PID Constants for the move motion (vertical motion)
		 * 
		 * @param pid the PIDConstants for this movement
		 */
		void setMovePIDConstants(PIDConstants pid);
		/*
		 * Sets the PID Constants for the turn motion (rotational motion)
		 *
		 * @param pid the PIDConstants for this movement
		 */
		void setTurnPIDConstants(PIDConstants pid);
		/*
		 * Sets the PID Constants for the swing motion (pivoting motion)
		 *
		 * @param pid the PIDConstants for this movement
		 */
		void setSwingPIDConstants(PIDConstants pid);
		/*
		 * Sets the accuracy value for how well the robot must face a point before moveving forward.
		 * Plays a key role in determining the speed of auton routine during Peer Persuit algorithm
		 *
		 * @param accuracy accuracy value (1 is avg)
		 */
		void setFacingAccuracy(double accuracy);
		/*
		 * Sets the PID Exit conditions for the move motion (vertical motion)
		 * Plays a key role in accuracy and speed of auton.
		 *
		 * @param error_limit the error from the goal which is tolerable.
		 * @param threshold_time amount of time the robot must be in the eror_limit to reach goal.
		 */
		void setMovePIDExitConditions(double error_limit, int threshold_time);
		/*
		 * Sets the PID Exit conditions for the turn motion (rotational motion)
		 * Plays a key role in accuracy and speed of auton.
		 *
		 * @param error_limit the error from the goal which is tolerable.
		 * @param threshold_time amount of time the robot must be in the eror_limit to reach goal.
		 */
		void setTurnPIDExitConditions(double error_limit, int threshold_time);
		/*
		 * Sets the PID Exit conditions for the turn motion (pivotal motion)
		 * Plays a key role in accuracy and speed of auton.
		 *
		 * @param error_limit the error from the goal which is tolerable.
		 * @param threshold_time amount of time the robot must be in the eror_limit to reach goal.
		 */
		void setSwingPIDExitConditions(double error_limit, int threshold_time);
		/*
		 * Moves the robot to the desired location. 
		 * This is the Absolute coordinate relative to starting position (0, 0) at the beginning of the program
		 *
		 * @param x x coordinate of new point
		 * @param y y coordinate of new point
		 */
		void moveTo(double x, double y);
		/*
		 * Moves the robot to the desired location.
		 * This is the Absolute coordinate relative to starting position (0, 0) at the beginning of the program
		 *
		 * @param x x coordinate of new point
		 * @param y y coordinate of new point
		 * @param timeout the amount of time the motion should take to complete. 
		 */
		void moveTo(double x, double y, int timeout);
		/*
		 * Moves the robot to the desired location.
		 * This is the Absolute coordinate relative to starting position (0, 0) at the beginning of the program
		 *
		 * @param x x coordinate of new point
		 * @param y y coordinate of new point
		 * @param timeout the amount of time the motion should take to complete.
		 * @param face_backward if true, will move to point with the back of the robot facing the point.
		 */
		void moveTo(double x, double y, int timeout, bool face_backward);
		/*
		 * reverses the heading of the bot.
		 * 
		 * If called, will toggle the facing such that the back or the front of the robot will face the other direction throughout the other movements.
		 */
		void reverseHeading();
		/*
		 * Turns the robot by a given amount of degrees relative to the current position.
		 *
		 * @param degrees amount of degrees to turn from current position.
		 */
		void turnRelative(int degrees);
		/*
		 * Turns the robot by a given amount of degrees in terms of the absolute position.
		 * The absolute position is the position relative to the starting angle of the program (0 degrees)
		 *
		 * @param degrees amount of degrees to turn from current position.
		 */
		void turnAbsolute(int degrees);
	};
};


#endif // CHASSIS_SE_H