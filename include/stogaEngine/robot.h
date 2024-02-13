/**
 * @file robot.h
 * @author Rishit Varshney
 * @brief This file contains the parent class of the important Robot class.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ROBOT_SE_H
#define ROBOT_SE_H

#include "drivetrain.h"
#include "tos.h"
#include "component.h"
#include "engine.h"

namespace engine {
class Bot {
protected: 
    void defaultDriveBehavior(pros::controller_analog_e_t move=pros::E_CONTROLLER_ANALOG_LEFT_Y, 
        pros::controller_analog_e_t turn=pros::E_CONTROLLER_ANALOG_RIGHT_X, int drift=5);
public:
    /**
     * @brief Controller Instance, make sure to initialize it.
     */
    engine::ControllerComponent master;
    /**
     * @brief Drivetrain instance... Initialize accordingly!
     * Add drivetrain motors (and odom wheels if using them)
     */
    engine::AbstractDrivetrain* drivetrain;
    /**
     * @brief TOS instance... Be careful while initializing this!
     */
    engine::AbstractTemporaryOdomSystem* odom;
    /**
     * @brief Instructions, feed into engine to get running auton. 
     */
    engine::Instructions ins;
    /**
     * @brief The auton engine of the bot.
     * Makes up the escence of all auton routines in Stoga Engine
     */
    engine::Engine* engine;
    /**
     * @brief All components for the bot...
     * Add auxillary items like catas, pistons, etc...
     */
    engine::ComponentList components;
    /**
     * @brief All sensors on the bot...
     * Add important sensors like IMU, vision, color, ETC...
     */
    engine::SensorComponentList sensors;

    /**
     * @brief Initializes the following fields...
     * 
     * initializes all instances.. except for drivetrain and odom.
     * They are programmer specific, so they are initialized in **setup()**
     */
    virtual void initInstances(std::vector<engine::Waypoint>& p);

    /**
     * @brief Initializes all instances of the robot...
     * Make sure to define this in inherited class.
     * Must call **initInstances()** method before anything
     * 
     * Register all compenents and sensors here.
     * Also initialize drivetrain and odom.
     * 
     * Also configure all settings. Use things from settings.h if nessecary.
     */
    virtual void setup() = 0;

    /**
     * @brief Method for drivecontrol.
     * 
     * This method is run in a repetitive loop.
     * Not nessecary to define, since bindings were setup in **setup()**
     * 
     * must call bind function for all components that are wanted to be used.
     */
    virtual void driveControl();

    /**
     * @brief Destroy the Bot object
     * Deletes odom & drivetrain
     */
    ~Bot();
};
};

#endif // ROBOT_SE_H
