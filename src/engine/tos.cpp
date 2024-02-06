#include "stogaEngine/tos.h"
using namespace engine::presets;

engine::Waypoint::Waypoint(std::string& c, double p1, double p2, double p3)
    : command(c), param1(p1), param2(p2), param3(p3) {}

IMUVectorOrientedTOS::IMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string id) {
    drivetrain = a;
    sensors = l;
    approach = engine::CartesianLine(0, 0, 0);
    negate = engine::CartesianLine(0, 0, 0);
    imuID = id;
    imu = sensors.findID(id);
}
 
std::array<double, 3> IMUVectorOrientedTOS::updateCoordinates(double odom_constant) {
    std::array<double, 3> tor;
    theta = imu->data1();
    tor[2] = imu->data1();

    rel_l = drivetrain->left_distance() - old_l;
	rel_r = drivetrain->right_distance() - old_r;

    double mag = (rel_l + rel_r) / 2;
    x += (mag * sin(tor[2])) * odom_constant;
    y += (mag * cos(tor[2])) * odom_constant;

    old_l = drivetrain->left_distance();
	old_r = drivetrain->right_distance();

    return {x, y};
}

static double calcSlope(double x1, double y1, double x2, double y2) {
    return (y2 - y1) / (x2 - x1);
}
static double angleDifference(double end, double start) {
    double red = (end-start);
    if (red > 180) {
        return red - 360;
    }
    return red;
}

void IMUVectorOrientedTOS::setBrakingConditions(double min_allowed_error, 
            double min_allowed_error_deg, 
            double error_time, 
            double error_timeout) {
    if (min_allowed_error != 0) MIN_ALLOWED_ERROR = min_allowed_error;
    if (min_allowed_error_deg != 0) MIN_ALLOWED_ERROR_DEG = min_allowed_error_deg;
    if (error_time != 0) ERROR_TIME = error_time;
    if (error_timeout != 0) ERROR_TIMEOUT = error_timeout;
}

void IMUVectorOrientedTOS::setApproachMode(bool pass) {is_pass = pass;}

void IMUVectorOrientedTOS::setInertialFactor(double val) {INERTIAL_FACTOR = val;}

std::array<double, 2> IMUVectorOrientedTOS::move(Waypoint& point) {
    if (point.command != "move") return std::array<double, 2>();
    double gx = point.param1 * sin(latest_turn); 
    double gy = point.param1 * cos(latest_turn);
    double error_turn = angleDifference(latest_turn, theta);

    double dx = gx - x;
    double dy = gy - y;
    double error = sqrt(dy*dy + dx*dx);

    approach.x = x;
    approach.y = y;
    approach.slope = calcSlope(x, y, gx, gy);
    negate.x = gx;
    negate.y = gy;
    negate.slope = negate.get_perp(approach.slope);
    if (negate.is_above(x, y)) error *= -1;

    double power = move_pid.update(error);
    double turn = turn_pid.update(error_turn);
    abs_timer++;

    if (power > 127) power = 127;
    else if (power < -127) power = -127;
    power *= abs(cos(INERTIAL_FACTOR * error_turn));
    
    std::array<double, 2> v = {power + turn, power - turn};

    if (abs(error) < MIN_ALLOWED_ERROR) {
        if (is_pass) timer = ERROR_TIME;
        timer++;
    }

    return v;
}

std::array<double, 2> IMUVectorOrientedTOS::power(Waypoint& point) {
    if (point.command != "power") return std::array<double, 2>();
    drivetrain->move(point.param1);
    pros::delay(point.param2);
    drivetrain->move(0);
    is_bashing = 1;

    return std::array<double, 2>();
}

std::array<double, 2> IMUVectorOrientedTOS::turn(Waypoint& point) {
    if (point.command != "turn") return std::array<double, 2>();
    latest_turn = point.param1;

    double error_turn = angleDifference(point.param1, theta);
    double turn = turn_pid.update(error_turn);
    std::array<double, 2> v = {turn, -turn};

    if (abs(error_turn) < MIN_ALLOWED_ERROR_DEG) {
        if (is_pass) timer = ERROR_TIME;
        timer++;
    }

    return v;
}

bool IMUVectorOrientedTOS::is_running() {
    if (is_bashing) return false;
    else return timer <= ERROR_TIME && abs_timer <= ERROR_TIMEOUT;
}

void IMUVectorOrientedTOS::reset(bool reset_cord) {
    move_pid.reset();
    turn_pid.reset();

    if (reset_cord) {
        old_l = 0;
        old_r = 0;
        rel_l = 0;
        rel_r = 0;

        x = 0;
        y = 0;
        theta = 0;
        drivetrain->reset();
        imu->reset();
    }

    timer = 0;
    abs_timer = 0;
    is_bashing = 0;
}

engine::AbstractTemporaryOdomSystem* engine::generateNewIMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string imuID) {
    return (engine::AbstractTemporaryOdomSystem*)(new engine::presets::IMUVectorOrientedTOS(a, l, imuID));
}
