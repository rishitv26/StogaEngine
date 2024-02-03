/**
 * @file robot.cpp
 * @author Rishit
 * 
 * @copyright Copyright (c) 2024
 * 
 * ========================================== Robot Class =========================================
 * This file contains the most important class, Robot. This class is responsible for all movements and sensory data collection.
 * This file should represent the real Robot well. There are presets provided in the library to use, or you can code your own!
 */

#include "stogaEngine/lib.h"
#include "settings.h"

class Robot : public engine::Bot {
private:
public:
    /*
     * Must call **initInstances()** method before anything
     * 
     * Register all compenents and sensors here.
     * Also initialize drivetrain and odom.
     * 
     * Also configure all settings. Use things from settings.h if nessecary.
     */
    void setup() {
        initInstances();
        
        drivetrain = (engine::AbstractDrivetrain*)(new engine::presets::TankDrivetrain(RIGHT_PORTS, LEFT_PORTS, 20));
        // odom = new [some class... lambda or preset];
        
        engine::MotorComponent m(9, "cata");
        components.registerNewComponent<engine::MotorComponent>(m);
        engine::MotorComponent intakeR(10, "right intake");
        components.registerNewComponent<engine::MotorComponent>(intakeR);
        engine::MotorComponent intakeL(11, "left intake");
        components.registerNewComponent<engine::MotorComponent>(intakeL);
        
        engine::PneumaticComponent p('A', "front wings");
        components.registerNewComponent<engine::PneumaticComponent>(p);
        engine::PneumaticComponent p2('B', "back wings");
        components.registerNewComponent<engine::PneumaticComponent>(p2);
        engine::PneumaticComponent blocker('C', "blocker");
        components.registerNewComponent<engine::PneumaticComponent>(blocker);
    }

    /**
     * @brief Drive control...
     * 
     * Code your opcontrol (or drive control) here. There is already a default behavior provided
     * such that it follows the desired binding set for each component in **setup()**
     */
    void driveControl() {
        defaultDriveBehavior();
    }

    ~Robot() {
        delete drivetrain;
        delete odom;
    }
};

