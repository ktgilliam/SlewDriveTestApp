#include "SlewDriveTest.h"

#include <fstream>
#include <chrono>
#include <ctime>
#include <cmath>


#include "config.h"

std::ofstream logFile;

int drvAID = 1;
int drvBID = 2;

double motorCommand = 0;

SlewDriveTest::SlewDriveTest()
{
    pDriveA = new KincoDriver(drvAID);
    pDriveB = new KincoDriver(drvBID);
    connected = false;
}

void SlewDriveTest::setupLogFile()
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

void SlewDriveTest::setupTerminal(TerminalInterface *_terminal)
{
    this->terminal = _terminal;

    if (TERMINAL_DRIVE_A)
        pDriveA->connectTerminalInterface(terminal);
    if (TERMINAL_DRIVE_B)
        pDriveB->connectTerminalInterface(terminal);

    terminal->printHeader();
}

bool SlewDriveTest::connectToDrivers(const char *devPath)
{
    bool result = true;
    try
    {
        KincoDriver::initializeRTU(devPath, 19200, 'N', 8, 1);
        result &= pDriveA->driverHandshake();
        result &= pDriveB->driverHandshake();
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what());
        exit(1);
    }
    connected = result;
    return connected;
}

bool SlewDriveTest::configureDriversForTest()
{
    try
    {
        pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);

        pDriveA->setDirectionMode(KINCO::CCW_IS_POSITIVE);
        pDriveB->setDirectionMode(KINCO::CW_IS_POSITIVE);

        // pDriveA->getPositionFeedback(TERMINAL_DRIVE_A);
        pDriveA->zeroPositionOffset();
        // pDriveA->getPositionFeedback(TERMINAL_DRIVE_A);

        // pDriveB->getPositionFeedback(TERMINAL_DRIVE_B);
        pDriveB->zeroPositionOffset();
        // pDriveB->getPositionFeedback(TERMINAL_DRIVE_B);

        collectFeedbackData();
        pDriveA->updateVelocityCommand(0.0);
        pDriveB->updateVelocityCommand(0.0);
        pDriveA->updateTorqueCommand(0.0);
        pDriveB->updateTorqueCommand(0.0);
        pDriveA->setDriverState(KINCO::POWER_ON_MOTOR);
        pDriveB->setDriverState(KINCO::POWER_ON_MOTOR);

#if OP_MODE == VELOCITY_MODE
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        // speedAmplitude = 100;
#elif OP_MODE == TORQUE_MODE

        pDriveA->setControlMode(KINCO::MOTOR_MODE_TORQUE);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);

        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        // auto maxSpeedIntA = KincoDriver::readDriverRegister<uint16_t>(drvAID, KINCO::MAX_SPEED);
        // auto maxSpeedIntB = KincoDriver::readDriverRegister<uint16_t>(drvBID, KINCO::MAX_SPEED);
        // pDriveB->setMaxSpeed(60);
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what());
    }
    return true;
}

void SlewDriveTest::updateCommands()
{
    static long int testCounter = 0;


    double N = (double)testCounter++;
    double arg = 2 * M_PI * T_s * N * f_0;
    double cmd = std::sin(arg);
#if OP_MODE == TORQUE_MODE
    motorCommand = torqueAmplitude * cmd;
    pDriveA->updateTorqueCommand(motorCommand);
    pDriveB->updateTorqueCommand(motorCommand);
#elif OP_MODE == VELOCITY_MODE
    motorCommand = speedAmplitude * cmd;
    pDriveA->updateVelocityCommand(motorCommand);
    pDriveB->updateVelocityCommand(motorCommand);
#endif
    collectFeedbackData();
}


void SlewDriveTest::logDeltaTime()
{
    static std::chrono::steady_clock::time_point prevTime;
    static bool firstTime = true;
    std::chrono::steady_clock::time_point nowTime = std::chrono::steady_clock::now();
    if (firstTime)
    {
        prevTime = nowTime;
        firstTime = false;
    }
    int32_t delta_us = std::chrono::duration_cast<std::chrono::microseconds>(nowTime - prevTime).count();
    prevTime = nowTime;
    if (delta_us < 0)
        throw std::runtime_error("this broke");
    std::string dtStr = std::to_string(delta_us);
    logFile << dtStr << ",";
}

void SlewDriveTest::collectFeedbackData()
{
    logDeltaTime();
    logFile << speedAmplitude * motorCommand << ",";
    logFile << pDriveA->getCurrentFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getCurrentFeedback(TERMINAL_DRIVE_B) << ",";
    logFile << pDriveA->getVelocityFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getVelocityFeedback(TERMINAL_DRIVE_B) << ",";
    logFile << pDriveA->getPositionFeedback(TERMINAL_DRIVE_A) << ",";
    logFile << pDriveB->getPositionFeedback(TERMINAL_DRIVE_B) << "\n";
}

void SlewDriveTest::shutdown()
{
    #if OP_MODE == TORQUE_MODE
    pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
    pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
#endif
    logFile.close();
    pDriveA->updateVelocityCommand(0);
    pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
    pDriveB->updateVelocityCommand(0);
    pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
}