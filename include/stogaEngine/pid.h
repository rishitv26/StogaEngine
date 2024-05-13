/**
 * @file chassis.h
 * @author Rishit Varshney
 * @brief This file contains all items related to PID algrothms.
 * @version 0.1
 * @date 2024-01-30
 *
 * @copyright Copyright (c) 2024
 *
 */


#ifndef PID_SE_H
#define PID_SE_H

namespace engine {
	/*
	* A structure that contains all PID constants.
	*/
	struct PIDConstants {
		double P;
		double I;
		double D;
		double I_range;
		/*
		* Construct the PID Constants accordingly.
		* 
		* @param p the proportional value
		* @param i the integral value
		* @param d the derivative value
		* @param irange the range from the goal under which the integral should have effect.
		*/
		explicit PIDConstants(double p, double i=0, double d=0, double irange=10);
		/*
		* Constructs the PID Constants to a default of P=0, I=0, D=0, and I Range=10.
		*/
		explicit PIDConstants();
	};
	
	/*
	* The PID class that performs all rudamentry computations for basic PID. Use this as an abstraction mechanism.
	*/
	class PID {
	private:
		PIDConstants constants;
		double value;
		double integral;
		double previous_error;
		double goal;
	public:
		/*
		* Constructs a PID object where all PID values are given via the PIDConstants object.
		* 
		* @param p the PID constants for this object.
		*/
		explicit PID(PIDConstants p);
		/*
		* Construct the PID Object with the given constants.
		*
		* @param p the proportional value
		* @param i the integral value
		* @param d the derivative value
		* @param irange the range from the goal under which the integral should have effect.
		*/
		explicit PID(double p, double i = 0, double d = 0, double irange = 10);
		/*
		* Sets the goal for the PID to evaluate the error for.
		* 
		* @param goal the new goal to reach to.
		*/
		void setGoal(double goal);
		/*
		* Gets the current goal this object is using to evaluate the error
		* 
		* @returns returns the current goal.
		*/
		double getGoal() const;
		/*
		* Updates the value from the PID algorithm based on a new sensor measurement.
		* 
		* @param sensor_value value measured from the sensor
		* @returns returns the new value from PID calculations
		*/
		double updateValue(double sensor_value);
		/*
		* Get the current value from the computation of the PID.
		* Will return a different value after the call updateValue();
		* 
		* @returns the value of the PID calculations.
		*/
		double getValue() const;
	};
};


#endif // PID_SE_H