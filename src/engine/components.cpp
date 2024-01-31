#include "stogaEngine/component.h"

void engine::ControllerComponent::initialize() {
    controller = new pros::Controller(pros::E_CONTROLLER_MASTER);
}

engine::Component& engine::ComponentList::operator[](size_t index) {
    if (index < cpp_vect.size()) {
        return *cpp_vect[index];
    } else throw ComponentIndexOutOfRange();
}

void engine::Component::bind(engine::binding_control_state b, engine::ControllerComponent& c) {
    if (b == binding_control_state::DEFAULT) {
        action(
            c.controller->get_digital(button1),
            c.controller->get_digital(button2),
            c.controller->get_analog(button3)
        );
    } else if (b == binding_control_state::REVERSE) {
        double one = !c.controller->get_digital(button1);
        double two = !c.controller->get_digital(button2);
        double three = -c.controller->get_analog(button3);
        action(one, two, three);
    } else if (b == binding_control_state::STICKY) {
        if (c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if (c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            button1_state,
            button2_state,
            c.controller->get_analog(button3)
        );
    } else if (b == binding_control_state::REVERSE_STICKY) {
        if (c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if (c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            !button1_state,
            !button2_state,
            -c.controller->get_analog(button3)
        );
    }
}

engine::ControllerComponent::~ControllerComponent() {
    delete controller;
};

void engine::ComponentList::addNewComponent(Component* c) {
    cpp_vect.push_back(c);
}

