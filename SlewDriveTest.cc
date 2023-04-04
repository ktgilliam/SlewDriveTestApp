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

#if OP_MODE == TORQUE_SIN_TEST_MODE || OP_MODE == VELOCITY_SIN_TEST_MODE
void SlewDriveTest::configureTest(SinTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    sinTestParamsPtr = paramsPtr;
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
#endif

#if OP_MODE == RAMP_TEST_MODE
void SlewDriveTest::configureTest(RampTestParams *const paramsPtr)
{
    if (paramsPtr == nullptr)
        throw std::runtime_error("configureTest:: nullptr");
    mysteryTestParamsPtr = paramsPtr;
    mysteryTestParamsPtr->ramp_step_counts = mysteryTestParamsPtr->ramp_duration * mysteryTestParamsPtr->F_s;
    mysteryTestParamsPtr->hold_step_counts = mysteryTestParamsPtr->hold_duration * mysteryTestParamsPtr->F_s;
    mysteryTestParamsPtr->stop_count = 2 * (2 * mysteryTestParamsPtr->ramp_step_counts + mysteryTestParamsPtr->hold_step_counts);
    mysteryTestParamsPtr->testSpeeds.reserve(mysteryTestParamsPtr->stop_count);

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
        // pDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        // pDriveB->setControlMode(KINCO::MOTOR_MODE_TORQUE);
    }
    catch (const std::exception &e)
    {
        terminal->addDebugMessage(e.what(), TERM::WARNING);
    }

    double ramp_dv = mysteryTestParamsPtr->max_speed / mysteryTestParamsPtr->ramp_step_counts;
    double curRampSpeed = 0.0;
#if BOTH_DIRECTIONS
    for (int ii = 0; ii < mysteryTestParamsPtr->ramp_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(curRampSpeed);
        curRampSpeed += ramp_dv;
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->hold_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(mysteryTestParamsPtr->max_speed);
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->ramp_step_counts * 2; ii++)
    {
        curRampSpeed -= ramp_dv;
        mysteryTestParamsPtr->testSpeeds.push_back(curRampSpeed);
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->hold_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(mysteryTestParamsPtr->max_speed * -1);
    }
    for (int ii = 0; ii < mysteryTestParamsPtr->ramp_step_counts; ii++)
    {
        mysteryTestParamsPtr->testSpeeds.push_back(curRampSpeed);
        curRampSpeed += ramp_dv;
    }
#else
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
#endif
}
#endif

void SlewDriveTest::updateCommands()
{
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
    collectFeedbackData();
    if (testCounter >= sinTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }
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
#elif OP_MODE == VELOCITY_SIN_TEST_MODE

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
    testCounter++;
}
#endif

#if OP_MODE == FRICTION_TEST_MODE
void SlewDriveTest::updateFrictionCommands()
{
    collectFeedbackData();
    if (testCounter >= frictionTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }

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
    collectFeedbackData();
    if (testCounter >= mysteryTestParamsPtr->stop_count)
    {
        testIsDone = true;
        return;
    }
    // if(testCounter <= mysteryTestParamsPtr->testSpeeds.size())
    motorACommand = mysteryTestParamsPtr->testSpeeds.at(testCounter);
    // else
    // return;
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
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(timeBuff, 32, "%Y_%m_%d_%H_%M_%OS", ptm);
    logPathSS << "/home/kevin/lfast_shared/SlewDriveTestLogs/" << testIdStr << timeBuff << ".csv";
    std::string logPathStr = logPathSS.str();
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
        logFile << pDriveA->getCurrentFeedback(updateConsole) << ",";
        logFile << pDriveB->getCurrentFeedback(updateConsole) << ",";
        logFile << pDriveA->getVelocityFeedback(updateConsole) << ",";
        logFile << pDriveB->getVelocityFeedback(updateConsole) << ",";
        logFile << pDriveA->getPositionFeedback(updateConsole) << ",";
        logFile << pDriveB->getPositionFeedback(updateConsole) << "\n";
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
