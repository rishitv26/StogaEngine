#include "stogaEngine/drivetrain.h"
#include "stogaEngine/errors.h"

pros::MotorGroup* engine::AbstractDrivetrain::getRightMotorGroup() {
    return right;
}

pros::MotorGroup* engine::AbstractDrivetrain::getLeftMotorGroup() {
    return left;
}

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

void TankDrivetrain::initialize(std::vector<int8_t> right_ports, std::vector<int8_t> left_ports, double circum, bool r) {
    wheel_circum = circum;
    reverse = r;
    if (r) {
        right = new pros::MotorGroup(right_ports);
        left = new pros::MotorGroup(negate(left_ports));
    }
    else {
        right = new pros::MotorGroup(negate(right_ports));
        left = new pros::MotorGroup(left_ports);
    }
    right->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    left->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void TankDrivetrain::move(int32_t analog) {
    if (right == nullptr || left == nullptr)
        throw UninitializedMotorError();
    right->move(analog);
    left->move(analog);
}

void TankDrivetrain::turn(int32_t analog) {
    if (right == nullptr || left == nullptr)
        throw UninitializedMotorError();
    right->move(analog);
    left->move(-analog);
}

void TankDrivetrain::reset() {
    if (right == nullptr || left == nullptr)
        throw UninitializedMotorError();
    right->tare_position_all();
    left->tare_position_all();
}

double TankDrivetrain::right_distance() {
    if (right == nullptr || left == nullptr)
        throw UninitializedMotorError();
    double norm;
    int i = 0;
    for (; i < right->size(); ++i) {
        norm += right->get_position(i);
    }
    norm /= i;
    return norm;
}

double TankDrivetrain::left_distance() {
    if (right == nullptr || left == nullptr)
        throw UninitializedMotorError();
    double norm;
    int i = 0;
    for (; i < left->size(); ++i) {
        norm += left->get_position(i);
    }
    norm /= i;
    return norm;
}
