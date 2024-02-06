#ifndef SCRIPTS_SE_H
#define SCRIPTS_SE_H

#include "robot.h"

namespace engine {

class Tester {
protected:
public:
    /**
     * @brief Runs the test
     * 
     * This class is meant to create automated scripts in order make life easier to test.
     * Make a constructor & deconstructor for child class and override this method to count as a test.
     * 
     * use **run()** to execute the test.
     * 
     * NOTE: use Pros API here only, or use presets instead.
     */
    virtual void run() = 0;
};

namespace presets {
class TestDriveTrain : public engine::Tester {
private:
    engine::Bot& robot;
public:
    TestDriveTrain(engine::Bot& robot);
    void run();
};

class GetLocationStats : public engine::Tester {
private:
    engine::Bot& robot;
    double odomConst;
public:
    GetLocationStats(engine::Bot& robot, double odom_const);
    void run();
};

class FindOdomConstant : public engine::Tester {
private:
    engine::Bot& robot;
    double odomConst;
public:
    FindOdomConstant(engine::Bot& robot);
    void run();
};

class FindPIDConstants : public engine::Tester {
private:
    engine::Bot& robot;
public:
    FindPIDConstants(engine::Bot& robot);
};
};

};

#endif // SCRIPTS_SE_H