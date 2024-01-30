#include "errors.h"
#include "api.h"
using namespace pros::lcd;

namespace engine {
    enum ErrorCodes {
        UNINITIALIZED_MOTOR = 300,
    };
}

inline UninitializedMotorError::UninitializedMotorError(char* message) {
    if (is_initialized()) {
       clear_line(0);
       clear_line(1);
       clear_line(2);
       clear_line(3);
       print(0, "PROGRAM TERMINATED...");
       print(1, "exit code SE%i", engine::ErrorCodes::UNINITIALIZED_MOTOR);
       print(2, message);
       print(3, "Uninitialized motor error.");
       exit(engine::ErrorCodes::UNINITIALIZED_MOTOR);
    }
}