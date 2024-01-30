#include "drivetrain.h"

engine::AbstractDrivetrain::~AbstractDrivetrain() {
    delete right;
    delete left;
}

static std::vector<int8_t> negate(std::vector<int8_t> v) {
    std::vector<int8_t> tor;
    for (int i = 0; i < v.size(); ++i) {
        tor.push_back(v[i]);
    }
    return tor;
}

void TankDrivetrain::initialize(std::vector<int8_t> right_ports, std::vector<int8_t> left_ports, bool r) {
    reverse = r;
    if (r) {
        right = new pros::MotorGroup(right_ports);
        left = new pros::MotorGroup(negate(left_ports));
    }
    else {
        right = new pros::MotorGroup(negate(right_ports));
        left = new pros::MotorGroup(left_ports);
    }
}

void TankDrivetrain::move(int32_t analog) {
    right->move(analog);
    left->move(analog);
}

void TankDrivetrain::turn(int32_t analog) {
    right->move(analog);
    left->move(-analog);
}