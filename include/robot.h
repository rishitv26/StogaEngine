#ifndef ROBOT_STOGAENGINE_H
#define ROBOT_STOGAENGINE_H

namespace engine {
class Robot {
public:
    Robot(){};
    virtual void initialize(Drivetrain drive) = 0;
};

};


#endif // ROBOT_STOGAENGINE_H