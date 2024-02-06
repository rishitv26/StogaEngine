#include "stogaEngine/calulations.h"
#include <math.h>

engine::PID::PID(double kp, double ki, double kd, double maxi, double mini)
    : KP(kp), KI(ki), KD(kd), prev_error(0), integral(0), MAX_INTEGRAL(maxi), MIN_INTEGRAL(mini) {}

void engine::PID::initialize(double kp, double ki, double kd, double maxi, double mini) {
    KP = kp;
    KI = ki;
    KD = kd;
    prev_error = 0;
    integral = 0;
    MAX_INTEGRAL = maxi;
    MIN_INTEGRAL = mini;
}

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

void engine::PID::reset() {
    integral = 0;
    prev_error = 0;
}

using namespace engine;

double CartesianLine::get_perp(double s)
{
    if (s * 0 != 0) return 0;
    return -1 / s;
}

CartesianLine::CartesianLine(double _slope, double _x, double _y)
{
    if (_slope * 0 != 0) y_cof = 0;
    slope = _slope;
    x = _x;
    y = _y;
}

double CartesianLine::get_slope() {
    return slope;
}

double CartesianLine::eval(double X) {
    return slope * (X - x) + y;
}

bool CartesianLine::is_on_line(double X, double Y) {
    return abs(eval(X) - Y) < 0.01;
};

bool CartesianLine::is_above(double X, double Y) {
    if (!y_cof) return false;
    double yVal = eval(X);
    return Y > yVal;
}

bool CartesianLine::is_bellow(double X, double Y) {
    if (!y_cof) return false;
    double yVal = eval(X);
    return Y < yVal;
}

