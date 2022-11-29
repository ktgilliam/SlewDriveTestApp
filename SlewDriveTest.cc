#include "SlewDriveTest.h"

#include <fstream>
#include <chrono>
#include <ctime>
#include <cmath>

std::ofstream logFile;

int drvAID = 1;
int drvBID = 2;

SlewDriveTest::SlewDriveTest()
{
    pDriveA = new KincoDriver(drvAID);
    pDriveB = new KincoDriver(drvBID);
    motorCommand = 0.0;
    connected = false;
    testConfigured = false;
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
        terminal->addDebugMessage(e.what(), TERM::ERROR);
        // exit(1);
    }
    connected = result;
    return connected;
}

bool SlewDriveTest::configureDrivers()
{
    try
    {
        pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);

        pDriveA->setDirectionMode(KINCO::CCW_IS_POSITIVE);
        pDriveB->setDirectionMode(KINCO::CW_IS_POSITIVE);
        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
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
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    return true;
}

void SlewDriveTest::configureTest(SinTestParams *const paramsPtr)
{
    sinTestParamsPtr = paramsPtr;
    activeTestType = SINUSOID_TEST;
    // stopCount = sinTestParamsPtr->num_prds * sinTestParamsPtr->prd_sec * sinTestParamsPtr->F_s;

    if (sinTestParamsPtr->mode == SinTestParams::TORQUE_MODE)
    {
        if (std::abs(sinTestParamsPtr->amplitude) > 1.0)
        {
            terminal->addDebugMessage("Torque amplitude exceeds limit. Setting to 100%", TERM::WARNING);
            sinTestParamsPtr->amplitude = 1.0;
        }
        try
        {
            pDriveA->setControlMode(KINCO::MOTOR_MODE_TORQUE);
            pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
            pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
            pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
    else if (sinTestParamsPtr->mode == SinTestParams::VELOCITY_MODE)
    {

        try
        {
            pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
            pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
}

void SlewDriveTest::updateSinCommands()
{
    static long int testCounter = 0;
    double N = (double)testCounter++;
    double T_s = 1.0 / sinTestParamsPtr->F_s;
    double f_0 = 1 / (double)sinTestParamsPtr->prd_sec;
    double arg = 2 * M_PI * T_s * N * f_0;
    double cmd = std::sin(arg);

    motorCommand = sinTestParamsPtr->amplitude * cmd;

    if (sinTestParamsPtr->mode == SinTestParams::TORQUE_MODE)
    {
        try
        {
            pDriveA->updateTorqueCommand(motorCommand);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
        try
        {
            pDriveB->updateTorqueCommand(motorCommand);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
    else if (sinTestParamsPtr->mode == SinTestParams::VELOCITY_MODE)
    {
        try
        {
            pDriveA->updateVelocityCommand(motorCommand);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
        try
        {
            pDriveB->updateVelocityCommand(motorCommand);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
    collectFeedbackData();
}

void SlewDriveTest::setupLogFile(const char *testIdStr)
{
    std::stringstream logPathSS;
    const auto now = std::time(NULL);
    const auto ptm = std::localtime(&now);
    char timeBuff[80];
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(timeBuff, 32, "%Y_%m_%d_%H_%M_%OS", ptm);
    logPathSS << "/home/kevin/Desktop/SlewDriveTestLogs/" << testIdStr << timeBuff << ".csv";
    std::string logPathStr = logPathSS.str();
    logFile.open(logPathStr.c_str(), std::ios::out);
    // print header row:
    logFile << "dt_uS, CMD,CURA,CURB,VELA,VELB,POSA,POSB\n";
}

void SlewDriveTest::collectFeedbackData()
{
    logDeltaTime();
    logFile << motorCommand << ",";
    try
    {
        logFile << pDriveA->getCurrentFeedback(true) << ",";
        logFile << pDriveB->getCurrentFeedback(true) << ",";
        logFile << pDriveA->getVelocityFeedback(true) << ",";
        logFile << pDriveB->getVelocityFeedback(true) << ",";
        logFile << pDriveA->getPositionFeedback(true) << ",";
        logFile << pDriveB->getPositionFeedback(true) << "\n";
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
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

void SlewDriveTest::setupTerminal(TerminalInterface *_terminal)
{
    this->terminal = _terminal;

    pDriveA->connectTerminalInterface(terminal);
    pDriveB->connectTerminalInterface(terminal);

    terminal->printHeader();
}

void SlewDriveTest::shutdown()
{
    logFile.close();

    try
    {
        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);

        pDriveA->updateVelocityCommand(0);
        pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pDriveB->updateVelocityCommand(0);
        pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
}
