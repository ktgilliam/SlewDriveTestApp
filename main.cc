// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>
#include <future>
#include <chrono>
#include <thread>
#include <cmath>

#include <mutex>
#include <condition_variable>
#include <deque>

#include "KinkoDriver.h"

constexpr int driveID = 1;

const int N = 4;
const int F_s = 10;
const int M = 3 * N * F_s;
const char devPath[] = "/dev/ttyUSB0";

// uint32_t posnVals[2][N];
// double velVals[2][N];
#define THREADED_TERMINAL 0

bool TERMINAL_DRIVE_A = true;
bool TERMINAL_DRIVE_B = true;

int drvAID = 1;
int drvBID = 2;
KinkoDriver *pDriveA;
KinkoDriver *pDriveB;
enum
{
    TORQUE_MODE,
    VELOCITY_MODE
};

uint8_t opMode = VELOCITY_MODE;

void collectFeedbackData()
{
    pDriveA->getCurrentFeedback(TERMINAL_DRIVE_A);
    pDriveB->getCurrentFeedback(TERMINAL_DRIVE_B);
    pDriveA->getVelocityFeedback(TERMINAL_DRIVE_A);
    pDriveB->getVelocityFeedback(TERMINAL_DRIVE_B);
    pDriveA->getPositionFeedback(TERMINAL_DRIVE_A);
    pDriveB->getPositionFeedback(TERMINAL_DRIVE_B);
}

int main()
{
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    pDriveA = new KinkoDriver(drvAID);
    pDriveB = new KinkoDriver(drvBID);

    if (TERMINAL_DRIVE_A)
        pDriveA->connectTerminalInterface(terminal);
    if (TERMINAL_DRIVE_B)
        pDriveB->connectTerminalInterface(terminal);
    terminal->printHeader();
    try
    {
        KinkoDriver::initializeRTU(devPath, 19200, 'N', 8, 1);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what());
        exit(1);
    }

    unsigned cnt = 0;
    unsigned switchCnt = 0;
    bool keepGoing = true;
    bool connected = pDriveA->driverHandshake();

    if (connected)
    {
        try
        {
            pDriveA->setDriverState(KINKO::POWER_OFF_MOTOR);
            pDriveB->setDriverState(KINKO::POWER_OFF_MOTOR);

            pDriveA->setDirectionMode(KINKO::CCW_IS_POSITIVE);
            pDriveB->setDirectionMode(KINKO::CW_IS_POSITIVE);

            pDriveA->getPositionFeedback(TERMINAL_DRIVE_A);
            pDriveA->zeroPositionOffset();
            pDriveA->getPositionFeedback(TERMINAL_DRIVE_A);

            pDriveB->getPositionFeedback(TERMINAL_DRIVE_B);
            pDriveB->zeroPositionOffset();
            pDriveB->getPositionFeedback(TERMINAL_DRIVE_B);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what());
        }

        pDriveA->updateVelocityCommand(0.0);
        pDriveB->updateVelocityCommand(0.0);
        pDriveA->updateTorqueCommand(0.0);
        pDriveB->updateTorqueCommand(0.0);
        pDriveA->setDriverState(KINKO::POWER_ON_MOTOR);
        pDriveB->setDriverState(KINKO::POWER_ON_MOTOR);

        const double torqueAmplitude = 1.5;
        ;
        const double speedAmplitude = KINKO::MOTOR_MAX_SPEED_RPM;
        ;
        if (opMode == VELOCITY_MODE)
        {
            pDriveA->setControlMode(KINKO::MOTOR_MODE_SPEED);
            pDriveB->setControlMode(KINKO::MOTOR_MODE_SPEED);
            // speedAmplitude = 100;
        }
        else if (opMode == TORQUE_MODE)
        {
            pDriveA->setControlMode(KINKO::MOTOR_MODE_TORQUE);
            pDriveB->setControlMode(KINKO::MOTOR_MODE_TORQUE);

            pDriveA->setMaxSpeed(KINKO::MOTOR_MAX_SPEED_RPM);
            pDriveB->setMaxSpeed(KINKO::MOTOR_MAX_SPEED_RPM);
            auto maxSpeedIntA = KinkoDriver::readDriverRegister<uint16_t>(drvAID, KINKO::MAX_SPEED);
            auto maxSpeedIntB = KinkoDriver::readDriverRegister<uint16_t>(drvBID, KINKO::MAX_SPEED);
            // pDriveB->setMaxSpeed(60);
        }
        double argHist[M]{0};
        static long int n = 0;
        try
        {
            while (keepGoing)
            {
                const double f_0 = N;
                const double T_s = 1/(double)F_s;
                double arg = 2 * M_PI * T_s * n++ / f_0;
                argHist[cnt] = arg;
                double cmd = std::sin(arg);
                if (opMode == TORQUE_MODE)
                {
                    pDriveA->updateTorqueCommand(torqueAmplitude * cmd);
                    pDriveB->updateTorqueCommand(torqueAmplitude * cmd);
                }
                else if (opMode == VELOCITY_MODE)
                {
                    pDriveA->updateVelocityCommand(speedAmplitude * cmd);
                    pDriveB->updateVelocityCommand(speedAmplitude * cmd);
                }
                int32_t slpPrd = (int32_t)T_s * 1000;
                std::this_thread::sleep_for(std::chrono::milliseconds(slpPrd));

                // Approx... doesn't account for preceding code

                collectFeedbackData();
                if(cnt++ >= M) keepGoing = false;
                // if (opMode == TORQUE_MODE)
                // {
                //     pDriveA->updateTorqueCommand(cmd_A / 2);
                //     pDriveB->updateTorqueCommand(cmd_A / 2);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                //     pDriveA->updateTorqueCommand(cmd_A / 3);
                //     pDriveB->updateTorqueCommand(cmd_A / 3);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                // }
            }
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what());
        }
    }
    if (opMode == TORQUE_MODE)
    {
        pDriveA->setMaxSpeed(KINKO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINKO::MOTOR_MAX_SPEED_RPM);
    }
    pDriveA->updateVelocityCommand(0);
    pDriveA->setDriverState(KINKO::POWER_OFF_MOTOR);
    pDriveB->updateVelocityCommand(0);
    pDriveB->setDriverState(KINKO::POWER_OFF_MOTOR);
    return 0;

    // std::unique_ptr<KinkoDriver>kinkoDrivePtr(new KinkoDriver(driveID, devPath));
    // kinkoDrivePtr->getControlMode();
}