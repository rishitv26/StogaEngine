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
class Component {
private:
public:
    /**
     * @brief Performs an action on this component. Uses 3 inputs 
     * 
     * @param analog1 analog1 value from [-127, 127]
     * @param analog2 analog2 value from [-127, 127]
     * @param analog3 analog3 value from [-127, 127]
     */
    virtual void action(int analog1, int analog2=0, int analog3=0);

    /**
     * @brief Binds the compenent to this control on the remote.
     * 
     * @param control remote controll button or joystick to bind to. 
     */
    virtual void bind(pros::controller_digital_e_t control);

    /**
     * @brief Stops the components from thier action, and releases them if they are in action
     */
    virtual void brake();
};

class ComponentList {
private:
    std::vector<Component> cpp_vect;
public:
    Component& operator[](size_t index);
};
};

#endif // COMPONENT_SE_H