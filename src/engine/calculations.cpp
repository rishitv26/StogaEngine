#include "stogaEngine/calulations.h"
#include <math.h>

engine::PID::PID(double kp, double ki, double kd, double maxi, double mini)
    : KP(kp), KI(ki), KD(kd), prev_error(0), integral(0), MAX_INTEGRAL(maxi), MIN_INTEGRAL(mini) {}

double engine::PID::update(double error) {
    double P = KP * error;
    integral += error;
    if (abs(error) < 0.01) integral = 0;
    if (integral > MAX_INTEGRAL || integral < MIN_INTEGRAL) integral = 0;
    double I = KI * integral;
    double D = KD * (error - prev_error);
    prev_error = error;
    return P + I + D;
}
