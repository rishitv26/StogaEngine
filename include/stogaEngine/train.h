/**
 * @file robot.h
 * @author Rishit Varshney
 * @brief This file contains all low level motor interactions to help with abstraction.
 * @version 0.1
 * @date 2024-05-13
 *
 * @copyright Copyright (c) 2024
 *
 */


#ifndef TRAIN_SE_H
#define TRAIN_SE_H

#include "data.h"
#include "../api.h"

namespace engine {
	/*
	* A set of motors with a common behaviour. For example: Left motors of Chassis.
	*/
	class Train {
	protected:
		pros::MotorGroup motors;
		int speedRPM;
		bool isSlewEnabled;
		Data<double> data;
		double oldDistance;

	public:
		Train();
		/*
		* Contructs the Train object.
		* copies the given motor group and stores it to manage.
		* 
		* @param groupProperties the MotorGroup representing the properties of this train.
		*/
		Train(pros::MotorGroup* groupProperties);
		/*
		* Sets the ports for this train.
		*
		* @param ports list of ports. Negate port number if motor is needed to be reversed
		*/
		void setPorts(const std::initializer_list<int> ports);
		/*
		* Gets the MotorGroup behind the object.
		* Returns a reference of the MotorGroup object.
		* 
		* @returns MotorGroup object
		*/
		pros::MotorGroup& getMotorGroup();
		/*
		* Sets the analog speed of all motors on the train.
		* Will sqeeze parameter in the range [-127, 127] if given value is outside this range.
		* 
		* @param value the analog value [-127, 127] to set speed of motor.
		*/
		virtual void setAnalog(int value);
		/*
		* Sets the voltage of all motors on the train.
		* Will sqeeze parameter in the range [-12000, 12000] if given value is outside this range.
		*
		* @param value the voltage (mV) value [-12000, 12000] to set speed of motor.
		*/
		virtual void setVoltage(int miliVolts);
		/*
		* Gets the ACTUALL speed of the motors in average.
		* 
		* @returns the actuall RPM of the motors.
		*/
		virtual double getSpeedRPM();
		/*
		* Turns on slew for all motors in Train.
		* Slew only affected positive accleration (not when its slowing down)
		* 
		* @param maxDelta the maximum change in analog speed [0, 127]
		*/
		virtual void enableSlew(int maxDelta);
		/*
		* Turns off slew for all motors in Train.
		*/
		virtual void disableSlew();
		/*
		* Gets the status of the motors and returns as a Data class.
		* The data includes telemetry, targets, and other things such as gearings.
		* 
		* @returns the data in terms of the Data class.
		*/
		virtual Data<double> getStatus();
		/*
		* Reverses the current direction for all motors.
		*/
		virtual void reverse();
		/*
		* After each call, this method calculates the change in distance
		* from the last time this method was called.
		* 
		* @returns the change in distance
		*/
		double newDistanceTraveled();
		/*
		* Sets the metrics of the measurements of distance from the motors.
		* TODO
		*/
		// void setMetric();
	};
};


#endif // !TRAIN_SE_H
