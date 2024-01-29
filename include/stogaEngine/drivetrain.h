#ifndef DRIVETRAIN_SE_H
#define DRIVETRAIN_SE_H
#include "vector"

namespace engine {
class Drivetrain {
protected:
public:
    /// @brief Default constructor. Does nothing...
    explicit Drivetrain() {};
    
    /**
     * @brief Moves the drivetrain by "analog"
    */
    virtual void move(int analog) = 0;

    virtual void turn(int analog) = 0;
};
};


#endif // DRIVETRAIN_SE_H