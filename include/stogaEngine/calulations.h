/**
 * @file calulations.h
 * @author Rishit Varshney
 * @brief Contains classes to compute odometry & PID related calculations
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CALCULATIONS_SE_H
#define CALCULATIONS_SE_H

namespace engine {
class PID {
private:
    double KP;
    double KI;
    double KD;
    double MAX_INTEGRAL;
    double MIN_INTEGRAL;

    double integral;
    double prev_error;
public:
    /**
     * @brief Construct a new PID object using KP, KI, KD
     * 
     * @param kp Proportion constant
     * @param ki Integral constant
     * @param kd Dervivative constant
     */
    PID(double kp, double ki=0, double kd=0, double max_i=100, double min_i=-100);

    /**
     * @brief Computes the PID value using error & constants
     * 
     * @param error error value... calculated
     * @return value
     */
    double update(double error);
};
};

#endif // CALCULATIONS_SE_H