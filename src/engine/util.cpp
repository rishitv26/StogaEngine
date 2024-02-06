#include "stogaEngine/util.h"
#include "api.h"

void sleep() {
    while (true) pros::delay(1000);
}

void __debug(int line, const char* file, const char* message) {
    pros::lcd::clear_line(0);
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);

    pros::lcd::print(0, "PROGRAM SET TO SLEEP...");
    pros::lcd::print(2, "invoked in %s, in line %i", file, line);
    pros::lcd::print(3, "-- message: %s", message);
}
