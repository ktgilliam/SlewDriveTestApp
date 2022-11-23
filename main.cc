// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <future>
#include <chrono>
#include <thread>
#include <cmath>

#include <chrono>
#include <ctime>

#include "KincoDriver.h"

const char devPath[] = "/dev/ttyUSB0";

#define TORQUE_MODE 0
#define VELOCITY_MOE 1

#define OP_MODE VELOCITY_MOE

#if OP_MODE == TORQUE_MODE
#define PRD_SEC 30
#define OP_MODE_STR "TRQ_"
#else
#define PRD_SEC 30
#define OP_MODE_STR "VEL_"
#endif

#define NUM_PERIODS 3
#define UPDATE_RATE_HZ 10

const int F_s = UPDATE_RATE_HZ;
constexpr int stop_count = NUM_PERIODS * PRD_SEC * F_s;
const unsigned int period_sec = PRD_SEC;

const double torqueAmplitude = 4.0;
const double speedAmplitude = KINCO::MOTOR_MAX_SPEED_RPM;

// uint32_t posnVals[2][N];
// double velVals[2][N];
#define THREADED_TERMINAL 0

bool TERMINAL_DRIVE_A = true;
bool TERMINAL_DRIVE_B = true;

int drvAID = 1;
int drvBID = 2;
KincoDriver *pDriveA;
KincoDriver *pDriveB;
std::ofstream logFile;

void logDeltaTime()
{
    static std::chrono::steady_clock::time_point prevTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point nowTime = std::chrono::steady_clock::now();
    std::string dtStr = std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(nowTime - prevTime).count());
    logFile << dtStr << ",";
    prevTime = nowTime;
}
void collectFeedbackData()
{
    logDeltaTime();
    logFile << pDriveA->getCurrentFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getCurrentFeedback(TERMINAL_DRIVE_B) << ",";
    logFile << pDriveA->getVelocityFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getVelocityFeedback(TERMINAL_DRIVE_B) << ",";
    logFile << pDriveA->getPositionFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getPositionFeedback(TERMINAL_DRIVE_B) << "\n";
}

void setupLogFile()
{
    std::stringstream logPathSS;
    const auto now = std::time(NULL);
    const auto ptm = std::localtime(&now);
    char timeBuff[80];
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(timeBuff, 32, "%Y_%m_%d_%H_%M_%OS", ptm);
    logPathSS << "/home/kevin/Desktop/SlewDriveTestLogs/" << OP_MODE_STR << timeBuff << ".csv";
    std::string logPathStr = logPathSS.str();
    logFile.open(logPathStr.c_str(), std::ios::out);
    // print header row:
    logFile << "dt_uS, CMD,CURA,CURB,VELA,VELB,POSA,POSB\n";
}


int main()
{
    setupLogFile();
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    pDriveA = new KincoDriver(drvAID);
    pDriveB = new KincoDriver(drvBID);

    if (TERMINAL_DRIVE_A)
        pDriveA->connectTerminalInterface(terminal);
    if (TERMINAL_DRIVE_B)
        pDriveB->connectTerminalInterface(terminal);
    terminal->printHeader();
    try
    {
        KincoDriver::initializeRTU(devPath, 19200, 'N', 8, 1);
        pDriveA->driverHandshake();
        pDriveB->driverHandshake();
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
            pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
            pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);

            pDriveA->setDirectionMode(KINCO::CCW_IS_POSITIVE);
            pDriveB->setDirectionMode(KINCO::CW_IS_POSITIVE);

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
        pDriveA->setDriverState(KINCO::POWER_ON_MOTOR);
        pDriveB->setDriverState(KINCO::POWER_ON_MOTOR);

#if OP_MODE == VELOCITY_MOE
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        // speedAmplitude = 100;
#elif OP_MODE == TORQUE_MODE

        pDriveA->setControlMode(KINCO::MOTOR_MODE_TORQUE);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);

        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        auto maxSpeedIntA = KincoDriver::readDriverRegister<uint16_t>(drvAID, KINCO::MAX_SPEED);
        auto maxSpeedIntB = KincoDriver::readDriverRegister<uint16_t>(drvBID, KINCO::MAX_SPEED);
        // pDriveB->setMaxSpeed(60);
#endif

        static long int n = 0;
        try
        {
            while (keepGoing)
            {
                constexpr double f_0 = 1 / (double)period_sec;
                constexpr double T_s = 1 / (double)F_s;
                double arg = 2 * M_PI * T_s * n++ * f_0;
                double cmd = std::sin(arg);

#if OP_MODE == TORQUE_MODE
                logFile << torqueAmplitude * cmd << ",";
                pDriveA->updateTorqueCommand(torqueAmplitude * cmd);
                pDriveB->updateTorqueCommand(torqueAmplitude * cmd);
#elif OP_MODE == VELOCITY_MOE
                logFile << speedAmplitude * cmd << ",";
                pDriveA->updateVelocityCommand(speedAmplitude * cmd);
                pDriveB->updateVelocityCommand(speedAmplitude * cmd);
#endif

                int32_t slpPrd = (int32_t)T_s * 1000;
                std::this_thread::sleep_for(std::chrono::milliseconds(slpPrd));

                // Approx... doesn't account for preceding code

                collectFeedbackData();
                if (cnt++ >= stop_count)
                    keepGoing = false;
            }
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what());
        }
    }
#if OP_MODE == TORQUE_MODE
    pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
    pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
#endif
    logFile.close();
    pDriveA->updateVelocityCommand(0);
    pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
    pDriveB->updateVelocityCommand(0);
    pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
    return 0;

    // std::unique_ptr<KincoDriver>kincoDrivePtr(new KincoDriver(driveID, devPath));
    // kincoDrivePtr->getControlMode();
}