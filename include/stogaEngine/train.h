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
		
		virtual int getSpeedRPM();
		
		virtual void enableSlew(int maxDelta);
		virtual void disableSlew();

		virtual Data<double> getStatus();

		virtual void reverse();
		double newDistanceTraveled();

		void setMetric();
	};

	class PerpendicularTrain : public Train {
		// TODO
	};

};


#endif // !TRAIN_SE_H
