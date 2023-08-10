#include "SlewDriveTest.h"
#include "config.h"
#include <fstream>
#include <chrono>
#include <ctime>
#include <cmath>

std::ofstream logFile;

// int drvAID = 1;
// int drvBID = 2;
int drvAID = DRIVER_A_ID;
int drvBID = DRIVER_B_ID;

unsigned testStopCount;

inline double frictionTorque(double speed_rpm)
{
    double coulomb = std::signbit(speed_rpm) ? -1 * COULOMB_FRICTION_NM : COULOMB_FRICTION_NM;
    double viscous = speed_rpm * VISCOUS_FRICTION_NM_PER_RPM;
    return coulomb + viscous;
}

SlewDriveTest::SlewDriveTest()
{
    pDriveA = new KincoDriver(drvAID);
    pDriveB = new KincoDriver(drvBID);
    motorACommand = 0.0;
    motorBCommand = 0.0;
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
    #ifndef DEBUG_MODE
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
    #else
    collectFeedbackData();
    terminal->addDebugMessage("Leaving configureDrivers()", TERM::WARNING);
    #endif
    return true;
}

#if OP_MODE == TORQUE_SIN_TEST_MODE || OP_MODE == VELOCITY_SIN_TEST_MODE
void SlewDriveTest::configureTest(SinTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    sinTestParamsPtr = paramsPtr;
    testStopCount = sinTestParamsPtr->stop_count;
    // stopCount = sinTestParamsPtr->num_prds * sinTestParamsPtr->prd_sec * sinTestParamsPtr->F_s;

#if OP_MODE == TORQUE_SIN_TEST_MODE
    if (std::abs(sinTestParamsPtr->amplitude) > 1.0)
    {
        terminal->addDebugMessage("Warning: Torque amplitude exceeds limit.", TERM::WARNING);
        // terminal->addDebugMessage("Torque amplitude exceeds limit. Setting to 100%", TERM::WARNING);
        // sinTestParamsPtr->amplitude = 1.0;
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
#elif OP_MODE == VELOCITY_SIN_TEST_MODE

    try
    {
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
#if MOTOR_B_MODE == MOTOR_VELOCITY_MODE
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
#elif MOTOR_B_MODE == MOTOR_TORQUE_MODE
        pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
#else
#error INVALID MOTOR CONFIGURATION
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }

#endif
}
#endif

#if OP_MODE == FRICTION_TEST_MODE
void SlewDriveTest::configureTest(FrictionTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    frictionTestParamsPtr = paramsPtr;
    int numSteps = (4 * frictionTestParamsPtr->steps_per_side);
    frictionTestParamsPtr->counts_per_step = frictionTestParamsPtr->step_duration * frictionTestParamsPtr->F_s;
    frictionTestParamsPtr->stop_count = (unsigned)(numSteps * frictionTestParamsPtr->counts_per_step);
    testStopCount = frictionTestParamsPtr->stop_count;

    try
    {
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
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
#endif

#if OP_MODE == RAMP_TEST_MODE
void SlewDriveTest::configureTest(RampTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    rampTestParamsPtr = paramsPtr;
    rampTestParamsPtr->ramp_step_counts = rampTestParamsPtr->ramp_duration * rampTestParamsPtr->F_s;
    rampTestParamsPtr->hold_step_counts = rampTestParamsPtr->hold_duration * rampTestParamsPtr->F_s;
    rampTestParamsPtr->stop_count = (2 * rampTestParamsPtr->ramp_step_counts) + rampTestParamsPtr->hold_step_counts;
#if BOTH_DIRECTIONS
    rampTestParamsPtr->stop_count = 2 * rampTestParamsPtr->stop_count;
#else
#endif
    testStopCount = rampTestParamsPtr->stop_count;
    rampTestParamsPtr->testCommand.reserve(rampTestParamsPtr->stop_count);
#ifndef DEBUG_MODE
    try
    {

#if MOTOR_A_MODE == MOTOR_VELOCITY_MODE
        pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
#elif MOTOR_A_MODE == MOTOR_TORQUE_MODE
        pDriveA->setControlMode(KINCO::MOTOR_MODE_TORQUE);
#else
#error INVALID MOTOR A CONFIGURATION
#endif

#if MOTOR_B_MODE == MOTOR_VELOCITY_MODE
        pDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
#elif MOTOR_B_MODE == MOTOR_TORQUE_MODE
        pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
#else
#error INVALID MOTOR B CONFIGURATION
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
#endif

    double ramp_dv = rampTestParamsPtr->test_ampl / rampTestParamsPtr->ramp_step_counts;
    double command = 0.0;
#if BOTH_DIRECTIONS
    // Ramp up (0 to max)
    for (int ii = 0; ii < rampTestParamsPtr->ramp_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(command);
        command += ramp_dv;
    }
    // Hold
    for (int ii = 0; ii < rampTestParamsPtr->hold_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(rampTestParamsPtr->test_ampl);
    }
    // Ramp down (max to -max)
    for (int ii = 0; ii < rampTestParamsPtr->ramp_step_counts*2; ii++)
    {
        command -= ramp_dv;
        rampTestParamsPtr->testCommand.push_back(command);
    }
    // Hold
    for (int ii = 0; ii < rampTestParamsPtr->hold_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(rampTestParamsPtr->test_ampl * -1);
    }
    // Ramp back to zero
    for (int ii = 0; ii < rampTestParamsPtr->ramp_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(command);
        command += ramp_dv;
    }
#else
    for (int ii = 0; ii < rampTestParamsPtr->ramp_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(command);
        command += ramp_dv;
    }
    for (int ii = 0; ii < rampTestParamsPtr->hold_step_counts; ii++)
    {
        rampTestParamsPtr->testCommand.push_back(rampTestParamsPtr->test_ampl);
    }
    for (int ii = 0; ii < rampTestParamsPtr->ramp_step_counts; ii++)
    {
        command -= ramp_dv;
        rampTestParamsPtr->testCommand.push_back(command);
    }
#endif
}
#endif

void SlewDriveTest::updateCommands()
{
    collectFeedbackData();
    if (testCounter >= testStopCount)
    {
        testIsDone = true;
        return;
    }
#if OP_MODE == VELOCITY_SIN_TEST_MODE || OP_MODE == TORQUE_SIN_TEST_MODE
    updateSinCommands();
#elif OP_MODE == FRICTION_TEST_MODE
    updateFrictionCommands();
#elif OP_MODE == RAMP_TEST_MODE
    updateRampTestCommands();
#endif
}

#if OP_MODE == TORQUE_SIN_TEST_MODE || OP_MODE == VELOCITY_SIN_TEST_MODE
void SlewDriveTest::updateSinCommands()
{
    double N = (double)testCounter;
    double T_s = 1.0 / sinTestParamsPtr->F_s;
    double f_0 = 1 / (double)sinTestParamsPtr->prd_sec;
    double arg = 2 * M_PI * T_s * N * f_0;
    double cmd = std::sin(arg);
    // if (cmd < 0) cmd = 0;
    motorACommand = sinTestParamsPtr->amplitude * cmd;
#if OP_MODE == TORQUE_SIN_TEST_MODE
    motorBCommand = motorACommand;
    try
    {
        pDriveA->updateTorqueCommand(motorACommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    try
    {
        pDriveB->updateTorqueCommand(motorBCommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
#elif OP_MODE == VELOCITY_SIN_TEST_MODE
    try
    {
        pDriveA->updateVelocityCommand(motorACommand);
        motorBCommand = motorACommand;
        pDriveB->updateVelocityCommand(motorBCommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
#if MOTOR_B_MODE == MOTOR_TORQUE_MODE
    try
    {

        double trqCmd = frictionTorque(motorACommand) * 0.5;
        pDriveB->updateTorqueCommand(trqCmd);#include <chrono>
#include <ctime>   
    try
    {
        pDriveA->updateVelocityCommand(motorACommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    try
    {
#if MOTOR_B_MODE == MOTOR_TORQUE_MODE
        double trqCmd = frictionTorque(motorACommand) * 0.5;
        pDriveB->updateTorqueCommand(trqCmd);
#elif MOTOR_B_MODE == MOTOR_VELOCITY_MODE
        motorBCommand = motorACommand;
        pDriveB->updateVelocityCommand(motorBCommand);
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
#endif
#endif
    testCounter++;
}

#endif

#if OP_MODE == FRICTION_TEST_MODE
void SlewDriveTest::updateFrictionCommands()
{
    unsigned step_idx = testCounter / frictionTestParamsPtr->counts_per_step;
    motorACommand = frictionTestParamsPtr->testSpeeds.at(step_idx);
    motorBCommand = motorACommand;
    try
    {
        // motorCommand = 50;
        pDriveA->updateVelocityCommand(motorACommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    try
    {
        // motorCommand = 0;
        pDriveB->updateVelocityCommand(motorBCommand);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    testCounter++;
}
#endif

#if OP_MODE == RAMP_TEST_MODE
void SlewDriveTest::updateRampTestCommands()
{
    // if(testCounter <= rampTestParamsPtr->testCommand.size())
    motorACommand = rampTestParamsPtr->testCommand.at(testCounter);
    motorBCommand = motorACommand;
#ifndef DEBUG_MODE
    try
    {
#if MOTOR_A_MODE == MOTOR_TORQUE_MODE
        pDriveA->updateTorqueCommand(motorACommand);
#elif MOTOR_A_MODE == MOTOR_VELOCITY_MODE
        pDriveA->updateVelocityCommand(motorACommand);
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    try
    {
#if MOTOR_B_MODE == MOTOR_TORQUE_MODE
        // double trqCmd = frictionTorque(motorBCommand) * 0.5;
        pDriveB->updateTorqueCommand(motorBCommand);
#elif MOTOR_B_MODE == MOTOR_VELOCITY_MODE
        pDriveB->updateVelocityCommand(motorBCommand);
#endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }
    #endif
    testCounter++;
}
#endif

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
    char dateBuff[80];
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(dateBuff, 32, "%Y_%m_%d", ptm);
    std::strftime(timeBuff, 32, "%Y_%m_%d_%H_%M_%OS", ptm);
    logPathSS << "/home/kevin/lfast_shared/SlewDriveTestLogs/"  << dateBuff << "/" << testIdStr << timeBuff << ".csv";
    logPathStr = logPathSS.str();
    logFile.open(logPathStr.c_str(), std::ios::out);
    // print header row:
    logFile << "dt_uS,CMDA,CMDB,CURA,CURB,VELA,VELB,POSA,POSB\n";
}

void SlewDriveTest::collectFeedbackData()
{
    logDeltaTime();
    static uint16_t cnt = 0;
    bool updateConsole = false;
    if (cnt++ >= CONSOLE_DOWNSAMPLE)
    {
        updateConsole = true;
        cnt = 0;
    }
    logFile << motorACommand << "," << motorBCommand << ",";
    try
    {
        #ifndef DEBUG_MODE
        logFile << pDriveA->getCurrentFeedback(updateConsole) << ",";
        logFile << pDriveB->getCurrentFeedback(updateConsole) << ",";
        logFile << pDriveA->getVelocityFeedback(updateConsole) << ",";
        logFile << pDriveB->getVelocityFeedback(updateConsole) << ",";
        logFile << pDriveA->getPositionFeedback(updateConsole) << ",";
        logFile << pDriveB->getPositionFeedback(updateConsole) << "\n";
        #else
        logFile  << "-1,-1,-1,-1,-1\n";
        static uint64_t cnt = 0;
        if (cnt++ %CONSOLE_DOWNSAMPLE == 0)
        {
            char msgBuff[40];
            sprintf(msgBuff, "Time = %4.2f", double(cnt)*UPDATE_PERIOD_SEC);
            terminal->addDebugMessage(msgBuff);
        }
        #endif
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
    int32_t delta_us = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - prevTime).count();
    prevTime = nowTime;
    if (delta_us < 0)
        throw std::runtime_error("this broke");
    std::string dtStr = std::to_string(delta_us);
    logFile << dtStr << ",";
}

void SlewDriveTest::setupTerminal(TerminalInterface *_terminal)
{
    this->terminal = _terminal;
#ifndef DEBUG_MODE
    pDriveA->connectTerminalInterface(terminal);
    pDriveB->connectTerminalInterface(terminal);
#endif
    terminal->printHeader();
}

void SlewDriveTest::shutdown()
{
    logFile.close();

    try
    {
        #ifndef DEBUG_MODE
        pDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);

        pDriveA->updateVelocityCommand(0);
        pDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pDriveB->updateVelocityCommand(0);
        pDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
        #endif
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }

    std::cout << VT100::CURSOR_TO_ROW_COL(50, 0);
    std::cout << logPathStr;
}
