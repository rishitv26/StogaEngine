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
     * @brief Construct a new PID object
     */
    PID() {}
    /**
     * @brief Construct a new PID object using KP, KI, KD
     * 
     * @param kp Proportion constant
     * @param ki Integral constant
     * @param kd Derivative constant
     * @param max_i Max value for integral
     * @param min_i Min value for integral
     */
    PID(double kp, double ki=0, double kd=0, double max_i=100, double min_i=-100);

    /**
     * @brief Non-default constructor for PID
     * 
     * @param kp Proportion constant
     * @param ki Integral constant
     * @param kd Derivative constant
     * @param max_i Max value for integral
     * @param min_i Min value for integral
     */
    void initialize(double kp, double ki=0, double kd=0, double max_i=100, double min_i=-100);

    /**
     * @brief Computes the PID value using error & constants
     * 
     * @param error error value... calculated
     * @return value
     */
    double update(double error);

    /**
     * @brief Reset PID values for nex iteration.
     * 
     */
    void reset();
};

class CartesianLine {
private:
    int y_cof = 1;
public:
    double x;
    double y;
    double slope;
    CartesianLine() {}
    /// @brief Gets the perpendicular slope of another slope
    /// @param s a slope
    /// @return the perpendicular counterpart
    static double get_perp(double s);
    /// @brief Default constructor...
    /// @param _slope slope wanted
    /// @param _x x coordinated to intersect
    /// @param _y y coordinated to intersect
    explicit CartesianLine(double _slope, double _x, double _y);
    /// @brief Gets the slope of the line
    /// @return Slope of the line
    double get_slope();
    /// @brief Evaluates the Y given the X.
    /// @param X x coord
    /// @return the coresponding Y coordinate.
    double eval(double X);
    /// @brief Checks if point given is on line...
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_on_line(double X, double Y);
    /// @brief Checks of point is above the line... (will return false is slope is UN)
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_above(double X, double Y);
    /// @brief Checks of point is below the line... (will return false is slope is UN)
    /// @param X x coord
    /// @param Y y coord
    /// @return boolean if condition satisfied.
    bool is_bellow(double X, double Y);
};
};

#endif // CALCULATIONS_SE_H