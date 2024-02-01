#include "stogaEngine/component.h"

void engine::ControllerComponent::initialize() {
    controller = new pros::Controller(pros::E_CONTROLLER_MASTER);
}

engine::Component& engine::ComponentList::operator[](size_t index) {
    if (index >= cpp_vect.size() || index < 0) {
        return *cpp_vect[index];
    } else throw ComponentIndexOutOfRange();
}

void engine::Component::bind(engine::ControllerComponent& c) {
    if (bstate == binding_control_state::DEFAULT) {
        action(
            (int)button1 == -1 ? 0 : c.controller->get_digital(button1),
            (int)button2 == -1 ? 0 : c.controller->get_digital(button2),
            (int)button3 == -1 ? 0 : c.controller->get_analog(button3)
        );
    } else if (bstate == binding_control_state::REVERSE) {
        double one = (int)button1 == -1 ? 0 : !c.controller->get_digital(button1);
        double two = (int)button2 == -1 ? 0 : !c.controller->get_digital(button2);
        double three = (int)button3 == -1 ? 0 : -c.controller->get_analog(button3);
        action(one, two, three);
    } else if (bstate == binding_control_state::STICKY) {
        if ((int)button1 == -1 ? 0 : c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if ((int)button2 == -1 ? 0 : c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            button1_state,
            button2_state,
            (int)button3 == -1 ? 0 : c.controller->get_analog(button3)
        );
    } else if (bstate == binding_control_state::REVERSE_STICKY) {
        if ((int)button1 == -1 ? 0 : c.controller->get_digital_new_press(button1)) button1_state = !button1_state;
        if ((int)button2 == -1 ? 0 : c.controller->get_digital_new_press(button2)) button1_state = !button1_state;
        action(
            !button1_state,
            !button2_state,
            (int)button3 == -1 ? 0 : -c.controller->get_analog(button3)
        );
    }
}

std::string engine::Component::stringId() {return "component";}

engine::ControllerComponent::~ControllerComponent() {
    delete controller;
};

void engine::Component::brake() {action(0);}

template <typename T>
void engine::ComponentList::registerNewComponent(T& c) {
    engine::Component* thing = (engine::Component*)(&c);
    try {
        if (thing->stringId() != "component") throw InvalidComponent();
    } catch {
        throw InvalidComponent();
    }
    cpp_vect.push_back(thing);
}

size_t engine::ComponentList::size() {
    return cpp_vect.size();
}

engine::SensorComponent& engine::SensorComponentList::operator[](size_t index) {
    if (index >= cpp_vect.size() || index < 0) {
        return *cpp_vect[index];
    } else throw SensorComponentIndexOutOfRange();
}

template <typename T>
void engine::SensorComponentList::registerNewSensorComponent(T& s) {
    engine::SensorComponent* thing = (engine::SensorComponent*)(&c);
    try {
        if (thing->stringId() != "sensor") throw InvalidSensorComponent();
    } catch {
        throw InvalidSensorComponent();
    }
    cpp_vect.push_back(thing);
}

size_t engine::SensorComponentList::size() {
    return cpp_vect.size();
}

void engine::SensorComponent::initialize(int8_t p) {
    port = p;
}

double engine::SensorComponent::data2() {
    return data1();
}

double engine::SensorComponent::data3() {
    return data1();
}

std::string engine::SensorComponent::stringId() {return "sensor";}

void engine::ComponentList::bindAll(ControllerComponent& c) {
    for (Component* i : cpp_vect) {
        i->bind(c);
    }
}

void engine::SensorComponentList::updateAll() {
    for (SensorComponent* i : cpp_vect) {
        i->data1();
        i->data2();
        i->data3();
    }
}

engine::PneumaticComponent::PneumaticComponent(
    int p, pros::controller_digital_e_t b1, 
    pros::controller_digital_e_t b2, 
    pros::controller_analog_e_t b3,
    engine::binding_control_state bs
) {
    piston = new pros::ADIDigitalOut(p);
    port = p;
    button1 = b1;
    button2 = b2;
    button3 = b3;
    bstate = bs;
}

void engine::PneumaticComponent::action(int analog1, int analog2, int analog3) {
    piston->set_value(analog1);
}

engine::PneumaticComponent::~PneumaticComponent() {delete piston;}

engine::MotorComponent::MotorComponent (
    int p, pros::controller_digital_e_t b1, 
    pros::controller_digital_e_t b2, 
    pros::controller_analog_e_t b3,
    engine::binding_control_state bs,
    bool r,
    pros::motor_brake_mode_e brake
) {
    motor = new pros::Motor(p);
    motor->set_brake_mode(brake);
    port = p;
    button1 = b1;
    button2 = b2;
    button3 = b3;
    bstate = bs;
    reverse = r;
}

void engine::MotorComponent::action(int analog1, int analog2, int analog3) {
    if (analog1 || analog2) {
        if (!reverse) motor->move_voltage(120000);
        else motor->move_voltage(120000);
    } else if (analog3 != 0) {
        if (!reverse) motor->move(analog3);
        else motor->move(-analog3);
    } else brake();
}

void engine::MotorComponent::brake() {motor->brake();}

void engine::ComponentList::haltAll() {
    for (int i = 0; i < size(); ++i) {
        cpp_vect[i]->brake();
    }
}
