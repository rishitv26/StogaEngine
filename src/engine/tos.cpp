#include "stogaEngine/tos.h"
using namespace engine::presets;

engine::Waypoint::Waypoint(std::string& c, double p1, double p2, double p3)
    : command(c), param1(p1), param2(p2), param3(p3) {}

IMUDifferentialDrive::IMUDifferentialDrive(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string id) {
    drivetrain = a;
    sensors = l;
    approach = engine::CartesianLine(0, 0, 0);
    negate = engine::CartesianLine(0, 0, 0);
    imuID = id;
    imu = sensors.findID(id);
}

///// ======================================================================================== TODO: 
std::array<double, 3> IMUDifferentialDrive::updateCoordinates(double odom_constant) {
    std::array<double, 3> tor;
    theta = imu->data1();
    tor[2] = imu->data1();

    rel_l = drivetrain->left_distance() - old_l;
	rel_r = drivetrain->right_distance() - old_r;
    rel_th = tor[2] - old_th;

    double mag = (rel_l + rel_r) / 2;
    // x += (mag * sin(tor[2])) * odom_constant;
    // y += (mag * cos(tor[2])) * odom_constant;
    x += mag * sin(rel_th);
    y += mag * cos(rel_th);

    tor[0] = x;
    tor[1] = y;

    old_l = drivetrain->left_distance();
	old_r = drivetrain->right_distance();
    old_th = tor[2];

    return tor;
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

void IMUDifferentialDrive::setBrakingConditions(double min_allowed_error, 
            double min_allowed_error_deg, 
            double error_time, 
            double error_timeout) {
    if (min_allowed_error != 0) MIN_ALLOWED_ERROR = min_allowed_error;
    if (min_allowed_error_deg != 0) MIN_ALLOWED_ERROR_DEG = min_allowed_error_deg;
    if (error_time != 0) ERROR_TIME = error_time;
    if (error_timeout != 0) ERROR_TIMEOUT = error_timeout;
}

void IMUDifferentialDrive::setApproachMode(bool pass) {is_pass = pass;}

void IMUDifferentialDrive::setInertialFactor(double val) {INERTIAL_FACTOR = val;}

bool IMUDifferentialDrive::move(Waypoint& point) {
    if (point.command != "move") return false;
    double error = 0;
    double error_turn = 0;
    std::array<double, 2> v;

    do {
        double gx = point.param1 * sin(latest_turn); 
        double gy = point.param1 * cos(latest_turn);
        error_turn = angleDifference(latest_turn, theta);

        double dx = gx - x;
        double dy = gy - y;
        error = sqrt(dy*dy + dx*dx);

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
        
        v = {power + turn, power - turn};
        drivetrain->set_raw_analog(v[0], v[1]);

        if (abs(error) < MIN_ALLOWED_ERROR) {
            if (is_pass) timer = ERROR_TIME;
            timer++;
        }


    } while (is_running());
    return true;
}

bool IMUDifferentialDrive::power(Waypoint& point) {
    
    if (point.command != "power") return false;
    drivetrain->move(point.param1);
    pros::delay(point.param2);
    drivetrain->move(0);
    is_bashing = 1;

    return true;
}

bool IMUDifferentialDrive::turn(Waypoint& point) {
    if (point.command != "turn") return false;
    latest_turn = point.param1;

    do {
        double error_turn = angleDifference(point.param1, theta);
        double turn = turn_pid.update(error_turn);
        std::array<double, 2> v = {turn, -turn};

        drivetrain->set_raw_analog(v[0], v[1]);

        if (abs(error_turn) < MIN_ALLOWED_ERROR_DEG) {
            if (is_pass) timer = ERROR_TIME;
            timer++;
        }
    } while (is_running());

    return true;
}

bool IMUDifferentialDrive::is_running() {
    if (is_bashing) return false;
    else return timer <= ERROR_TIME && abs_timer <= ERROR_TIMEOUT;
}

void IMUDifferentialDrive::reset(bool reset_cord) {
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

engine::AbstractTemporaryOdomSystem* engine::generateNewIMUDifferentialDrive(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string imuID) {
    return (engine::AbstractTemporaryOdomSystem*)(new engine::presets::IMUDifferentialDrive(a, l, imuID));
}
