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
     * Returns an empty const char* string...
     * @return const char* 
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

#endif // ERROR_SE_H