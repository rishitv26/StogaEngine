#include "stogaEngine/errors.h"
#include "api.h"
using namespace pros::lcd;

explicit engine::AbstractErrorHandler::AbstractErrorHandler() {
    display();
};

void engine::AbstractErrorHandler::display() {
    clear_line(0);
    clear_line(1);
    clear_line(2);
    clear_line(3);
    if (is_initialized()) {
        print(0, "Program Terminated");
        print(1, "exit code: SE%i", error_type);
        print(2, "-----------------------");
    } else {
        printf("Program Terminated\n");
        printf("exit code: SE%i\n", error_type);
        printf("-----------------------\n");
    }
    exit(error_type);
}

const char* engine::AbstractErrorHandler::what() {
    return ("SE" + std::to_string(error_type)).c_str();
}
