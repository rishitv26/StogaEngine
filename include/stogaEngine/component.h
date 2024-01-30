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

namespace engine {
class Component {
private:
public:
    
};

class ComponentList {
private:
    std::vector<Component> cpp_vect;
public:
    Component& operator[](size_t index) {
        if (index < cpp_vect.size()) {
            return cpp_vect[index];
        } else throw ComponentIndexOutOfRange();
    }
};
};

#endif // COMPONENT_SE_H