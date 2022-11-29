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

const char devPath[] = "/dev/ttyUSB0";

int main()
{
    SlewDriveTest *testObj = new SlewDriveTest();

    testObj->setupLogFile();
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    testObj->setupTerminal(terminal);
    bool connected = testObj->connectToDrivers(devPath);
    testObj->configureDriversForTest();
    
    unsigned cnt = 0;
    unsigned switchCnt = 0;
    bool keepGoing = true;

    if (connected)
    {
        static long int n = 0;
        try
        {
            while (keepGoing)
            {
                // testObj->testUpdate();
                std::thread tu = testObj->testUpdate();
                int32_t slpPrd = (int32_t)T_s * 1000;
                std::this_thread::sleep_for(std::chrono::milliseconds(slpPrd));
                tu.join();
                if (cnt++ >= stop_count)
                    keepGoing = false;
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