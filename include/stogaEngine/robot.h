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

namespace engine {
class Bot {
public:

    engine::ControllerComponent master;
    /**
     * @brief Drivetrain instance... Initialize accordingly!
     */
    engine::AbstractDrivetrain drivetrain;
    /**
     * @brief TOS instance... Be careful while initializing this!
     */
    engine::AbstractTemporaryOdomSystem odom;
    /**
     * @brief All components for the bot...
     * Add auxillary items like catas, pistons, etc...
     */
    engine::ComponentList components;

    /**
     * @brief Initializes the following fields...
     */
    void init();
};
};

#endif // ROBOT_SE_H
