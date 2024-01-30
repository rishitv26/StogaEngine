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

class Robot {
public:
    /**
     * @brief Drivetrain instance... Initialize accordingly!
     */
    engine::AbstractDrivetrain drivetrain;
    /**
     * @brief TOS instance... Be careful while initializing
     */
    engine::AbstractTemporaryOdomSystem odom;
    
    
};

#endif // ROBOT_SE_H
