#include "stogaEngine/component.h"

engine::Component& engine::ComponentList::operator[](size_t index) {
    if (index < cpp_vect.size()) {
        return cpp_vect[index];
    } else throw ComponentIndexOutOfRange();
}





