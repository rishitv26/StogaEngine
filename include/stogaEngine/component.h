/**
 * @file component.h
 * @author Rishit Varshney
 * @brief contains component class.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef COMPONENT_SE_H
#define COMPONENT_SE_H

#include <vector>
#include "stogaEngine/errors.h"
#include "api.h"

// all abstract classes...
namespace engine {
enum class binding_control_state {
    DEFAULT = 0,
    REVERSE = 1,
    STICKY = 2,
    REVERSE_STICKY = 3,
};

class ControllerComponent {
private:
public:
    pros::Controller* controller;    
    /**
     * @brief Non-default constructor for the controller...
     * Class is very similar to pros::Controller... all methods are availible
     */
    void initialize();
    /**
     * @brief Destroy the Controller Component object
     */
    ~ControllerComponent();
};

class Component {
protected:
    pros::controller_digital_e_t button1;
    pros::controller_digital_e_t button2;
    pros::controller_analog_e_t button3;
    engine::binding_control_state bstate;

    int button1_state = 0;
    int button2_state = 0;

    int8_t port;
public:
    explicit Component() {}
    /**
     * @brief Construct a new Component object with port and remote binding...
     * 
     * @param p port [0, 21]
     * @param b1 button to bind (all digital buttons)
     * @param b2 button to bind (all digital buttons)
     * @param b3 joystick / analog to bind (all analog joystick values)
     */
    explicit Component(int8_t p, pros::controller_digital_e_t b1=(pros::controller_digital_e_t)(-1), 
            pros::controller_digital_e_t b2=(pros::controller_digital_e_t)(-1), 
            pros::controller_analog_e_t b3=(pros::controller_analog_e_t)(-1),
            engine::binding_control_state bs=binding_control_state::DEFAULT)
                : button1(b1), button2(b2), button3(b3), port(p), bstate(bs) {}
    /**
     * @brief Binds the compenent to this control on the remote.
     * 
     * With the button provided, this will check if button is pressed and act accordingly.
     * Run through an iterative loop. 
     * @param c controller instance
     */
    void bind(ControllerComponent& c);
    
    /**
     * @brief Performs an action on this component. Uses 3 inputs 
     * 
     * @param analog1 analog1 value from [-127, 127]
     * @param analog2 analog2 value from [-127, 127]
     * @param analog3 analog3 value from [-127, 127]
     */
    virtual void action(int analog1, int analog2=0, int analog3=0) = 0;

    /**
     * @brief Stops the components from thier action, and releases them if they are in action
     */
    virtual void brake();

    /**
     * @brief Returns the "id" of the current component.
     * 
     * An ID is a way to differentiate different types of components. return any string that helps to identify this component.
     * @return std::string 
     */
    virtual std::string stringId() = 0;
};

class SensorComponent {
protected:
    int8_t port;
public:
    /**
     * @brief Non-default contructor... Construct accordingly
     * 
     * Initialize the sensor properly.
     * @param p port [0, 21]
     * @param ... can change depending on child.
     */
    virtual void initialize(int8_t p, ...);
    
    /**
     * @brief Reset the values of the sensor...
     */
    virtual void reset() = 0;

    /**
     * @brief Return 1st value of sensor.\
     * will UPDATE then return
     * @return double sensor value
     */
    virtual double data1() = 0;

    /**
     * @brief Return 2nd value of sensor (optional)
     * will UPDATE then return
     * @return double sensor value
     */
    virtual double data2();
    
    /**
     * @brief Return 3rd value of sensor (optional)
     * will UPDATE then return
     * @return double sensor value
     */
    virtual double data3();
};

class ComponentList {
private:
    std::vector<Component*> cpp_vect;
public:
    Component& operator[](size_t index);
    template <typename T>
    void registerNewComponent(T& c);
    
    size_t size();
    void bindAll(ControllerComponent& c);
};

class SensorComponentList {
private:
    std::vector<SensorComponent*> cpp_vect;
public:
    SensorComponent& operator[](size_t index);
    void registerNewSensorComponent(SensorComponent* c);
    size_t size();

    void updateAll();
};
};

// built in components:
namespace engine {
class PneumaticComponent : Component {
private:
    pros::ADIDigitalOut* piston;
public:
    PneumaticComponent(int p, pros::controller_digital_e_t b1=(pros::controller_digital_e_t)(-1), 
            pros::controller_digital_e_t b2=(pros::controller_digital_e_t)(-1), 
            pros::controller_analog_e_t b3=(pros::controller_analog_e_t)(-1),
            engine::binding_control_state bs=binding_control_state::DEFAULT);
    // only analog1 is used
    void action(int analog1, int analog2=0, int analog3=0);
    std::string stringId();
    ~PneumaticComponent();
};

class MotorComponent : Component {
private:
    pros::Motor* motor;
    bool reverse;
public:
    MotorComponent(int p, pros::controller_digital_e_t b1=(pros::controller_digital_e_t)(-1), 
        pros::controller_digital_e_t b2=(pros::controller_digital_e_t)(-1), 
        pros::controller_analog_e_t b3=(pros::controller_analog_e_t)(-1),
        engine::binding_control_state bs=binding_control_state::DEFAULT,
        bool reverse=false,
        pros::motor_brake_mode_e brake=pros::E_MOTOR_BRAKE_COAST);
    void action(int analog1, int analog2=0, int analog3=0);
    void brake();
    std::string stringId();
    ~MotorComponent();
};
};

#endif // COMPONENT_SE_H