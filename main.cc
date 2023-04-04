// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>

#include <stdexcept>
#include <future>
#include <chrono>
#include <thread>

#include <chrono>   
#include <ctime>

#include "config.h"
#include "SlewDriveTest.h"
#include "KincoDriver.h"
#include "TerminalInterface.h"

const char devPath[] = "/dev/ttyUSB0";

int main()
{
    SlewDriveTest *testObj = new SlewDriveTest();

    testObj->setupLogFile(OP_MODE_STR);
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    testObj->setupTerminal(terminal);
    bool connected = testObj->connectToDrivers(devPath);
    // connected = true;
    testObj->configureDrivers();

#if OP_MODE == TORQUE_SIN_TEST_MODE
    SinTestParams *sinTestParams = new SinTestParams(TORQUE_AMPLITUDE, PRD_SEC, NUM_PERIODS, UPDATE_RATE_HZ, TORQUE_SIN_TEST_MODE);
    testObj->configureTest(sinTestParams);
#elif OP_MODE == VELOCITY_SIN_TEST_MODE
    SinTestParams *sinTestParams = new SinTestParams(SPEED_AMPLITUDE, PRD_SEC, NUM_PERIODS, UPDATE_RATE_HZ, VELOCITY_SIN_TEST_MODE);
    testObj->configureTest(sinTestParams);
#elif OP_MODE == FRICTION_TEST_MODE
    FrictionTestParams *frictionTestParams = new FrictionTestParams(MAX_SPEED, STEPS_PER_SIDE, STEP_DURATION, UPDATE_RATE_HZ);
    testObj->configureTest(frictionTestParams);

#elif OP_MODE == RAMP_TEST_MODE
    RampTestParams *rampTestParams = new RampTestParams(MAX_SPEED, RAMP_DURATION, HOLD_DURATION, UPDATE_RATE_HZ);
    testObj->configureTest(rampTestParams);
#endif

    bool keepGoing = true;

    if (connected)
    {
        static long int n = 0;
        try
        {
            while (keepGoing)
            {
                // testObj->testUpdate();
                constexpr double T_s = 1.0 / UPDATE_RATE_HZ;
                constexpr int32_t slpPrd_us = (int32_t)T_s * 1000000;
                std::thread tu = testObj->testUpdate();
                std::this_thread::sleep_for(std::chrono::microseconds(slpPrd_us));
                tu.join();
                keepGoing = !testObj->testComplete();
            }
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what());
        }
    }

    testObj->shutdown();

    return 0;
}