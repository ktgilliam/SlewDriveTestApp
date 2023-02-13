#include "SlewDriveTest.h"
#include "config.h"
#include <fstream>
#include <chrono>
#include <ctime>
#include <cmath>

std::ofstream logFile;

int drvAID = 1;
int drvBID = 2;

inline double frictionTorque(double speed_rpm)
{
    double coulomb = std::signbit(speed_rpm) ? -1 * COULOMB_FRICTION_NM : COULOMB_FRICTION_NM;
    double viscous = speed_rpm * VISCOUS_FRICTION_NMPERRPM;
    return coulomb + viscous;
}

SlewDriveTest::SlewDriveTest()
{
    pDriveA = new KincoDriver(drvAID);
    pDriveB = new KincoDriver(drvBID);
    motorCommand = 0.0;
    connected = false;
    testConfigured = false;
    testCounter = 0;
    testIsDone = false;
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
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    sinTestParamsPtr = paramsPtr;
    activeTestType = SINUSOID_TEST;
    // stopCount = sinTestParamsPtr->num_prds * sinTestParamsPtr->prd_sec * sinTestParamsPtr->F_s;

    if (sinTestParamsPtr->mode == TORQUE_MODE)
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
    else if (sinTestParamsPtr->mode == VELOCITY_MODE)
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
    else if (sinTestParamsPtr->mode == MIXED_MODE)
    {

        try
        {
            pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
            pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
            pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
}

void SlewDriveTest::configureTest(FrictionTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    frictionTestParamsPtr = paramsPtr;
    activeTestType = FRICTION_TEST;

    int numSteps = (4 * frictionTestParamsPtr->steps_per_side);
    frictionTestParamsPtr->counts_per_step = frictionTestParamsPtr->step_duration * frictionTestParamsPtr->F_s;
    frictionTestParamsPtr->stop_count = (unsigned)(numSteps * frictionTestParamsPtr->counts_per_step);

    try
    {
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }

    frictionTestParamsPtr->testSpeeds.reserve(numSteps);
    double speedStepSize = frictionTestParamsPtr->max_speed / (double)(frictionTestParamsPtr->steps_per_side);

    // first step at zero
    frictionTestParamsPtr->testSpeeds.push_back(0.0);

    // negative, speeding up
    for (int ii = 1; ii <= frictionTestParamsPtr->steps_per_side; ii++)
    {
        double step_speed = (double)ii * speedStepSize * -1.0;
        frictionTestParamsPtr->testSpeeds.push_back(step_speed);
    }
    // negative, slowing down
    for (int ii = frictionTestParamsPtr->steps_per_side - 1; ii >= 0; ii--)
    {
        double step_speed = (double)ii * speedStepSize * -1.0;
        frictionTestParamsPtr->testSpeeds.push_back(step_speed);
    }
    // positive, speeding up
    for (int ii = 1; ii <= frictionTestParamsPtr->steps_per_side; ii++)
    {
        double step_speed = (double)ii * speedStepSize;
        frictionTestParamsPtr->testSpeeds.push_back(step_speed);
    }
    // positive, slowing down
    for (int ii = frictionTestParamsPtr->steps_per_side - 1; ii >= 0; ii--)
    {
        double step_speed = (double)ii * speedStepSize;
        frictionTestParamsPtr->testSpeeds.push_back(step_speed);
    }
}

void SlewDriveTest::configureTest(MysteryTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    mysteryTestParamsPtr = paramsPtr;
    activeTestType = MYSTERY_TEST;
    mysteryTestParamsPtr->ramp_step_counts = mysteryTestParamsPtr->ramp_duration * mysteryTestParamsPtr->F_s;
    mysteryTestParamsPtr->hold_step_counts = mysteryTestParamsPtr->hold_duration * mysteryTestParamsPtr->F_s;
    mysteryTestParamsPtr->stop_count = 2 * mysteryTestParamsPtr->ramp_step_counts + mysteryTestParamsPtr->hold_step_counts;
    mysteryTestParamsPtr->testSpeeds.reserve(mysteryTestParamsPtr->stop_count);

    double ramp_dv = mysteryTestParamsPtr->max_speed / mysteryTestParamsPtr->ramp_step_counts;
    double curRampSpeed = 0.0;
    for (int ii = 0; ii < mysteryTestParamsPtr->ramp_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(curRampSpeed);
        curRampSpeed += ramp_dv;
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->hold_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(mysteryTestParamsPtr->max_speed);
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->ramp_step_counts; ii++)
    {
        curRampSpeed -= ramp_dv;
        mysteryTestParamsPtr->testSpeeds.push_back(curRampSpeed);
    }
}
void SlewDriveTest::updateCommands()
{
    switch (activeTestType)
    {
    case SINUSOID_TEST:
        updateSinCommands();
        break;
    case FRICTION_TEST:
        updateFrictionCommands();
        break;
    case MYSTERY_TEST:
        updateMysteryCommands();
        break;
    }
}

void SlewDriveTest::updateSinCommands()
{
    if (testCounter++ >= sinTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }
    double N = (double)testCounter;
    double T_s = 1.0 / sinTestParamsPtr->F_s;
    double f_0 = 1 / (double)sinTestParamsPtr->prd_sec;
    double arg = 2 * M_PI * T_s * N * f_0;
    double cmd = std::sin(arg);

    motorCommand = sinTestParamsPtr->amplitude * cmd;

    if (sinTestParamsPtr->mode == TORQUE_MODE)
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
    else if (sinTestParamsPtr->mode == VELOCITY_MODE)
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
    else if (sinTestParamsPtr->mode == MIXED_MODE)
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
            double trqCmd = frictionTorque(motorCommand) * 0.5;
            pDriveB->updateTorqueCommand(trqCmd);
        }
        catch (const std::exception &e)
        {
            terminal->addDebugMessage(e.what(), TERM::WARNING);
        }
    }
    collectFeedbackData();
}

void SlewDriveTest::updateFrictionCommands()
{
    if (testCounter++ >= frictionTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }

    unsigned step_idx = testCounter / frictionTestParamsPtr->counts_per_step;
    motorCommand = frictionTestParamsPtr->testSpeeds.at(step_idx);
    try
    {
        // motorCommand = 50;
        pDriveA->updateVelocityCommand(motorCommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    try
    {
        // motorCommand = 0;
        pDriveB->updateVelocityCommand(motorCommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    collectFeedbackData();
}

void SlewDriveTest::updateMysteryCommands()
{
    if (testCounter >= mysteryTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }

    motorCommand = mysteryTestParamsPtr->testSpeeds.at(testCounter++);

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
        double trqCmd = frictionTorque(motorCommand) * 0.5;
        pDriveB->updateTorqueCommand(trqCmd);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    collectFeedbackData();
}

bool SlewDriveTest::testComplete()
{
    return testIsDone;
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

    std::cout << VT100::CURSOR_TO_ROW_COL(50, 0);
}
