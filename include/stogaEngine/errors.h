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

class UninitializedMotorError : public std::exception {
public:
    inline UninitializedMotorError(char* message = "");
};

#endif // ERROR_SE_H