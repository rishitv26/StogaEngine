/**
 * @file tos.h
 * @author Rishit Varshney
 * @brief File contains temporary odometry algorithms... Used for movements during autons.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef TOS_SE_H
#define TOS_SE_H

#include <string>
#include <array>
#include "drivetrain.h"
#include "component.h"
#include "calulations.h"

namespace engine {
struct Waypoint {
    std::string command = "";
    double param1 = 0;
    double param2 = 0;
    double param3 = 0;

    Waypoint(std::string& c, double p1=0, double p2=0, double p3=0);
    Waypoint() {}
};

class AbstractTemporaryOdomSystem {
protected:
public:
    /**
     * @brief recalculates coordinates and updates x, y, and theta
     * 
     * Returns the new updated coordinates in form of (x, y, theta).
     * Call in an iterative loop (might not be nessaccary)... and update accordingly in code.
     * 
     * @param helper_constant helps in most odom algorithms
     */
    virtual std::array<double, 3> updateCoordinates(double helper_constant) = 0;

    /**
     * @brief calculates the PID for moving forward / backward.
     * 
     * Use information about the current goal and the location of the bot...
     * Should not block any code...
     * Skip if instruction is incorrect (if command is bullcrap, and not move, skip)
     * 
     * @param point waypoint to reach...
     * @return std::array<double, 2> 
     */
    virtual std::array<double, 2> move(Waypoint& point) = 0;

    /**
     * @brief calculates the PID for turning clockwise / counterclockwise.
     * 
     * Use information about the current goal and the location of the bot...
     * Should not block any code... 
     * Skip if instruction is incorrect (if command is bullcrap, and not turn, skip)
     * 
     * @param point waypoint to reach...
     * @return std::array<double, 2> 
     */
    virtual std::array<double, 2> turn(Waypoint& point) = 0;

    /**
     * @brief Powers foreward without PID regulation
     * 
     * Use information about timing...
     * Can block code if nessecary, must notify **is_running()** method upon completion... 
     * Skip if instruction is incorrect (if command is bullcrap, and not power, skip)
     * 
     * @param point waypoint to reach...
     * @return std::array<double, 2>
     */
    virtual std::array<double, 2> power(Waypoint& point) = 0;

    /**
     * @brief Checks if PID is running...
     * 
     * if any form of movement or PID is still running (either in turn or move), return true
     * Power also counts as a "motion" function.
     * Keep in mind this function checks for motion in the **Drivetrain**
     * 
     * @return true motion command is still running...
     * @return false motion command is complete.
     */
    virtual bool is_running() = 0;

    /**
     * @brief Resets parameters for PID and other nesseccary items.
     * 
     * if some things need to be reset after each iteration (like PID stuff)
     * reset them here.
     * 
     * @param setting depending on main class, this can be helpful
     */
    virtual void reset(bool setting) = 0;
};

namespace presets {
class IMUVectorOrientedTOS : public AbstractTemporaryOdomSystem {
private:
    engine::AbstractDrivetrain* drivetrain;
    engine::SensorComponentList sensors;
    std::string imuID;
    engine::SensorComponent* imu;

    double rel_l = 0;
    double rel_r = 0;
    double old_l = 0;
    double old_r = 0;
    double theta = 0;

    double latest_turn = 0;

    double x = 0, y = 0;

    engine::PID move_pid;
    engine::PID turn_pid;

    engine::CartesianLine approach;
    engine::CartesianLine negate;

    double MIN_ALLOWED_ERROR = 1;
    double MIN_ALLOWED_ERROR_DEG = 1; 
    double ERROR_TIME = 100;
    double ERROR_TIMEOUT = 3000;

    bool is_pass = false;

    double timer = 0;
    double abs_timer = 0;

    double INERTIAL_FACTOR = 1;
    bool is_bashing = 0;
public:
    IMUVectorOrientedTOS():approach(engine::CartesianLine(0, 0, 0)), negate(engine::CartesianLine(0, 0, 0)) {}
    IMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string imuID);
    void setBrakingConditions(double min_allowed_error=0, 
            double min_allowed_error_deg=0, 
            double error_time=0, 
            double error_timeout=0);
    void setApproachMode(bool pass=false);
    void setInertialFactor(double val);
    std::array<double, 3> updateCoordinates(double odom_constant);
    std::array<double, 2> move(Waypoint& point);
    std::array<double, 2> turn(Waypoint& point);
    std::array<double, 2> power(Waypoint& point);
    void reset(bool reset_cord=false);
    bool is_running();
};
};

engine::AbstractTemporaryOdomSystem* generateNewIMUVectorOrientedTOS(AbstractDrivetrain* a, engine::SensorComponentList& l, std::string imuID);
};

#endif // TOS_SE_H