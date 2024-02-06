#include "stogaEngine/scripts.h"
#include "stogaEngine/util.h"
using namespace engine::presets;

#define ARE_SAME(X, Y) abs(X - Y) < 1

static double get_avg(std::vector<double> v) {
    double sum = 0;
    int i = 0;
    for (; i < v.size(); ++i) {
        sum += v[i];
    }
    return sum / i;
}

TestDriveTrain::TestDriveTrain(engine::Bot& r):robot(r) {}

void TestDriveTrain::run() {
    robot.master.controller->clear();
    robot.master.controller->print(0, 0, "Lift robot up");
    pros::delay(1000);
    for (int i = 5; i >= 0; --i) {
        robot.master.controller->print(1, 0, "Starting in: %i", i);
        pros::delay(1000);
    }
    robot.master.controller->clear();
    robot.drivetrain->getRightMotorGroup()->move_voltage(120000);
    robot.drivetrain->getLeftMotorGroup()->move_voltage(120000);
    bool speedr = false;
    double max_r = 0;
    bool speedl = false;
    double max_l = 0;
    bool equal = false;
    for (int i = 0; i < 10000; ++i) {
        double _r = get_avg(robot.drivetrain->getRightMotorGroup()->get_actual_velocity_all());
        double _l = get_avg(robot.drivetrain->getLeftMotorGroup()->get_actual_velocity_all());
        if (ARE_SAME(abs(_r), 600)) {
            speedr = true;
            if (_r > max_r) max_r = _r;
        }
        if (ARE_SAME(abs(_l), 600)) {
            speedl = true;
            if (_l > max_l) max_l = _l;
        }
        if (ARE_SAME(_r, _l)) {
            equal = true;
        }
        pros::delay(1);

        if (robot.master.controller->get_digital(pros::E_CONTROLLER_DIGITAL_A)) break;
    }
    robot.drivetrain->move(0);
    robot.master.controller->clear();
    pros::lcd::print(0, "R RPM: %i %s", (int)max_r, speedr ? "pass" : "fail");
    pros::lcd::print(1, "L RPM: %i %s", (int)max_l, speedl ? "pass" : "fail");
    pros::lcd::print(2, "Speed E: %s", equal ? "pass" : "fail");
    sleep();
}

GetLocationStats::GetLocationStats(engine::Bot& r, double odom_const)
    : robot(r), odomConst(odom_const) {}

void GetLocationStats::run() {
    pros::lcd::clear_line(0);
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    std::array<double, 3> cords;

    while (true) {
        cords = robot.odom->updateCoordinates(odomConst);
        pros::lcd::print(0, "(%f, %f)", cords[0], cords[1]);
        pros::lcd::print(1, "theta: %f", cords[2]);
        pros::lcd::print(2, "press A to reset...");

        if (robot.master.controller->get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            robot.master.controller->clear();
            robot.master.controller->print(0, 0, "(%i,%i):%i", (int)cords[0], (int)cords[1], (int)cords[2]);
            robot.odom->reset(true);
        }
        pros::delay(2);
    }
}

FindOdomConstant::FindOdomConstant(engine::Bot& r):robot(r) {}

void FindOdomConstant::run() {
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "please move the bot vertically by 10in...");
    pros::lcd::clear_line(1);
    pros::lcd::print(1, "press A upon completion.");
    double y_val = 0;
    while (!robot.master.controller->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        y_val = robot.odom->updateCoordinates(1)[1];
        pros::delay(2);
    }
    double odom_const = 10.0 / y_val;
    pros::lcd::clear_line(0);
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::clear_line(3);
    pros::lcd::print(0, "RESULTS...");
    pros::lcd::print(2, "I predict you constant to be:");
    pros::lcd::print(3, "%f", odom_const);
    sleep();
}

FindPIDConstants::FindPIDConstants(engine::Bot& r): robot(r) {}
