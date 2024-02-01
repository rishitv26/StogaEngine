#include "stogaEngine/component.h"

void engine::ControllerComponent::initialize() {
    controller = new pros::Controller(pros::E_CONTROLLER_MASTER);
}

engine::Component& engine::ComponentList::operator[](size_t index) {
    if (index >= cpp_vect.size() || index < 0) {
        return *cpp_vect[index];
    } else throw ComponentIndexOutOfRange();
}

void engine::Component::bind(engine::ControllerComponent& c, engine::binding_control_state b) {
    if (b == binding_control_state::DEFAULT) {
        action(
            (int)button1 == -1 ? 0 : c.controller->get_digital(button1),
            (int)button2 == -1 ? 0 : c.controller->get_digital(button2),
            (int)button3 == -1 ? 0 : c.controller->get_analog(button3)
        );
    } else if (b == binding_control_state::REVERSE) {
        double one = (int)button1 == -1 ? 0 : !c.controller->get_digital(button1);
        double two = (int)button2 == -1 ? 0 : !c.controller->get_digital(button2);
        double three = (int)button3 == -1 ? 0 : -c.controller->get_analog(button3);
        action(one, two, three);
    } else if (b == binding_control_state::STICKY) {
        if ((int)button1 == -1 ? 0 : c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if ((int)button2 == -1 ? 0 : c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            button1_state,
            button2_state,
            (int)button3 == -1 ? 0 : c.controller->get_analog(button3)
        );
    } else if (b == binding_control_state::REVERSE_STICKY) {
        if ((int)button1 == -1 ? 0 : c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if ((int)button2 == -1 ? 0 : c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            !button1_state,
            !button2_state,
            (int)button3 == -1 ? 0 : -c.controller->get_analog(button3)
        );
    }
}

engine::ControllerComponent::~ControllerComponent() {
    delete controller;
};

void engine::ComponentList::addNewComponent(Component* c) {
    cpp_vect.push_back(c);
}

size_t engine::ComponentList::size() {
    return cpp_vect.size();
}

engine::SensorComponent& engine::SensorComponentList::operator[](size_t index) {
    if (index >= cpp_vect.size() || index < 0) {
        return *cpp_vect[index];
    } else throw SensorComponentIndexOutOfRange();
}

void engine::SensorComponentList::addNewSensorComponent(SensorComponent* s) {
    cpp_vect.push_back(s);
}

size_t engine::SensorComponentList::size() {
    return cpp_vect.size();
}

void engine::SensorComponent::initialize(int8_t p, ...) {
    port = p;
}

double engine::SensorComponent::data2() {
    return data1();
}

double engine::SensorComponent::data3() {
    return data1();
}
