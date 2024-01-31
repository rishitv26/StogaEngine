#include "stogaEngine/calulations.h"

engine::PID::PID(double kp, double ki=0, double kd=0)
    : KP(kp), KI(ki), KD(kd), prev_error(0), integral(0) {}

double engine::PID::update(double error) {
    double P = KP * error;
    integral += error;
    double I = KI * integral;
    double D = KD * (error - prev_error);

    return P + I + D;
}
