#include "stogaEngine/pid.h"

engine::PIDConstants::PIDConstants(double p, double i, double d, double irange)
	:P(p), I(i), D(d), I_range(irange) {}

engine::PIDConstants::PIDConstants(): P(0), I(0), D(0), I_range(10) {}

engine::PID::PID(double p, double i, double d, double irange)
{
	constants = PIDConstants(p, i, d, irange);
	value = 0;
	integral = 0;
	previous_error = 0;
	goal = 0;
}

engine::PID::PID(PIDConstants pid)
{
	constants = pid;
	value = 0;
	integral = 0;
	previous_error = 0;
	goal = 0;
}

void engine::PID::setGoal(double newGoal) {
	goal = newGoal;
}

double engine::PID::getGoal() const {
	return goal;
}

double engine::PID::updateValue(double sensorValue) {
	// calculate error:
	double error = goal - sensorValue;
	
	// calculate P:
	value = constants.P * error;

	// calculate I:
	if (error < 0.001 && error > 0.001) {
		integral = 0;
	}
	else if (error > -constants.I_range && error < constants.I_range) {
		integral += error;
		value += constants.I * integral;
	}
	
	// calculate D:
	value += constants.D * (error - previous_error);
	previous_error = error;

	return value;
}

double engine::PID::getValue() const {
	return value;
}