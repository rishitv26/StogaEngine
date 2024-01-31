/**
 * @file component.h
 * @author Rishit Varshney
 * @brief contains component class.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef COMPONENT_SE_H
#define COMPONENT_SE_H
#include <vector>
#include "stogaEngine/errors.h"
#include "api.h"

namespace engine {

enum class binding_control_state {
    DEFAULT = 0,
    REVERSE = 1,
    STICKY = 2,
    REVERSE_STICKY = 3,
};

class ControllerComponent {
private:
public:
    pros::Controller* controller;    
    /**
     * @brief Non-default constructor for the controller...
     * Class is very similar to pros::Controller... all methods are availible
     */
    void initialize();
    /**
     * @brief Destroy the Controller Component object
     */
    ~ControllerComponent();
};

class Component {
protected:
    pros::controller_digital_e_t button1;
    pros::controller_digital_e_t button2;
    pros::controller_analog_e_t button3;

    int button1_state = 0;
    int button2_state = 0;
public:
    /**
     * @brief Construct a new Component object
     */
    explicit Component(pros::controller_digital_e_t b1, 
            pros::controller_digital_e_t b2=(pros::controller_digital_e_t)(-1), 
            pros::controller_analog_e_t b3=(pros::controller_analog_e_t)(-1) )
                : button1(b1), button2(b2), button3(b3) {}
    /**
     * @brief Binds the compenent to this control on the remote.
     * 
     * With the button provided, this will check if button is pressed and act accordingly.
     * Run through an iterative loop.
     */
    void bind(binding_control_state b=binding_control_state::DEFAULT, ControllerComponent& c);

    /**
     * @brief Performs an action on this component. Uses 3 inputs 
     * 
     * @param analog1 analog1 value from [-127, 127]
     * @param analog2 analog2 value from [-127, 127]
     * @param analog3 analog3 value from [-127, 127]
     */
    virtual void action(int analog1, int analog2=0, int analog3=0);

    /**
     * @brief Stops the components from thier action, and releases them if they are in action
     */
    virtual void brake();
};

class ComponentList {
private:
    std::vector<Component*> cpp_vect;
public:
    Component& operator[](size_t index);
    void addNewComponent(Component* c);
};
};

#endif // COMPONENT_SE_H