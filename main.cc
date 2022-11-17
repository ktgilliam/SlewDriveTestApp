// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>

#include "KincoDriver.h"

constexpr int driveID = 1;

const int N = 10;
const char devPath[] = "/dev/ttyUSB0";

double velVals[N]{0};

int main()
{
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    KincoDriver *kincoDrivePtr = new KincoDriver(1);

    kincoDrivePtr->connectTerminalInterface(terminal);
    terminal->printHeader();

    try
    {
        kincoDrivePtr->connectRTU(devPath, 19200, 'N', 8, 1);
        kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ENABLE);
        kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ON);
        kincoDrivePtr->setControlMode(KINKO::MOTOR_MODE_SPEED);
        kincoDrivePtr->updateVelocityCommand(200);
        unsigned long int cnt = 0;
        // int cnt = 0;
        // while (cnt++ < N)
        // {
        //     //

        //     // std::cout << velVals[cnt] << ",\t";
        // }
        bool keepGoing = true;
        while (keepGoing)
        {
            velVals[cnt++] = kincoDrivePtr->getVelocityFeedback(true);
            terminal->serviceCLI();
        }

        kincoDrivePtr->updateVelocityCommand(0);
        kincoDrivePtr->setDriverState(KINKO::MOTOR_STATE_ENABLE);

        std::cout << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << std::endl
                  << e.what() << '\n';
        std::cout << "ERROR";
    }

    // std::unique_ptr<KincoDriver>kincoDrivePtr(new KincoDriver(driveID, devPath));
    // kincoDrivePtr->getControlMode();

    delete kincoDrivePtr;
    delete terminal;
}