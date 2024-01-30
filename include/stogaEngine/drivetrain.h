/**
 * @file drivetrain.h
 * @author Rishit Varshney
 * @brief This file contains the interfaces for the drivetrain class, including thier presets.
 * @version 0.1
 * @date 2024-01-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef DRIVETRAIN_SE_H
#define DRIVETRAIN_SE_H
#include "vector"
#include "api.h"

namespace engine {
class AbstractDrivetrain {
protected:
    pros::MotorGroup* right = nullptr;
    pros::MotorGroup* left = nullptr;
public:
    /// @brief Default constructor. Does nothing...
    explicit AbstractDrivetrain() {};
    
    /**
     * @brief Moves the drivetrain by "analog"
     * This function moves by setting the motors by an analog value between [-127, 127]
     * Function must limit itself if analog is outside the given range (This might not be too important)
     * Note that the folowing is the default behaviour of a normal pros::MotorGroup or pros::Motor.
     * Must throw error on the screen if motor class is not initialized, and must terminate movement if any motor is too hot.
     *  
     * @param analog analog values to move drivetrain
    */
    virtual void move(int32_t analog);

    /**
     * @brief Turns drive train by "analog"
     * This function turns by setting the motors by an analog value between [-127, 127]
     * Function must limit itself if analog is outside the given range (This might not be too important)
     * Note that the folowing is the default behaviour of a normal pros::MotorGroup or pros::Motor.
     * Must throw error on the screen if motor class is not initialized, and must terminate movement if any motor is too hot.
     * 
     * @param analog analog values to turn drivetrain
    */
    virtual void turn(int32_t analog);

    /**
     * @brief Default deconstructor... Edit if new heap variables are added.
     */
    ~AbstractDrivetrain();
};
};

class TankDrivetrain : engine::AbstractDrivetrain {
protected:
    bool reverse;
public:
    explicit TankDrivetrain() {};
    
    void initialize(std::vector<int8_t> right_ports, std::vector<int8_t> left_ports, bool reverse=false);
    void move(int32_t analog);
    void turn(int32_t analog);
};

#endif // DRIVETRAIN_SE_H