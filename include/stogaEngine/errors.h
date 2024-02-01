/**
 * @file errors.h
 * @author Rishit Varshney
 * @brief Contains proper error classes that will give user nice indication of errors.
 * @version 0.1
 * @date 2024-01-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef ERRORS_SE_H
#define ERRORS_SE_H
#include <iostream>


namespace engine {
/**
 * @brief Contains all error codes...
 * This enum contains all error code for all errors that print automatically to the console.
 */
enum ErrorCodes {
    UNINITIALIZED_MOTOR = 300,
    COMPONENT_OUT_OF_BOUND = 301,
    SENSOR_COMPONENT_OUT_OF_BOUND = 302,
    INVALID_COMPONENT = 303,
    INVALID_SENSOR_COMPONENT = 304,
};

class AbstractErrorHandler : public std::exception {
protected:
    engine::ErrorCodes error_type;
    void display();
public:
    /**
     * @brief Construct a new Abstract Error Handler object.
     * 
     * @param message Optional message to print out...
     */
    explicit AbstractErrorHandler();
    /**
     * @brief Inherited function from std::exception...
     * Will not get a chance to return anything...
     * In case, will return error code as string. 
     */
    const char* what();
};
};

class UninitializedMotorError : public engine::AbstractErrorHandler {
protected:
    engine::ErrorCodes error_type = engine::UNINITIALIZED_MOTOR;
};

class ComponentIndexOutOfRange : public engine::AbstractErrorHandler {
protected:
    engine::ErrorCodes error_type = engine::COMPONENT_OUT_OF_BOUND;
};

class SensorComponentIndexOutOfRange : public engine::AbstractErrorHandler {
protected:
    engine::ErrorCodes error_type = engine::SENSOR_COMPONENT_OUT_OF_BOUND;
};

class InvalidComponent : public engine::AbstractErrorHandler {
    engine::ErrorCodes error_type = engine::INVALID_COMPONENT;
};

class InvalidSensorComponent : public engine::AbstractErrorHandler {
    engine::ErrorCodes error_type = engine::INVALID_SENSOR_COMPONENT;
};

#endif // ERROR_SE_H