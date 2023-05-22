#include "KincoDriver.h"
#include <thread>
#include <vector>
#include <string>
// enum
// {
//     SINUSOID_TEST,
//     FRICTION_TEST,
//     JSSG_2006,
//     RAMP_TEST,
// };
// enum
// {
//     TORQUE_MODE,
//     VELOCITY_MODE,
//     // MIXED_MODE
// };
struct TestParams
{
    unsigned stop_count;
};
struct SinTestParams
{
    SinTestParams(double _amplitude, double _prd_sec, unsigned _num_prds, unsigned _F_s, uint8_t _mode)
        : amplitude(_amplitude),
          prd_sec(_prd_sec),
          num_prds(_num_prds),
          F_s(_F_s),
        //   type(_type),
          mode(_mode)
    {
        stop_count = (unsigned)(_num_prds * _prd_sec * _F_s);
    };

    double amplitude;
    double prd_sec;
    unsigned num_prds;
    unsigned F_s;
    uint8_t type;
    uint8_t mode;
    unsigned stop_count;
};

struct FrictionTestParams
{
    FrictionTestParams(double _max_speed, unsigned _steps_per_side, double _step_duration, unsigned _F_s)
        : max_speed(_max_speed), steps_per_side(_steps_per_side), step_duration(_step_duration), F_s(_F_s) { }
    double max_speed;
    unsigned F_s;
    unsigned int steps_per_side;
    double step_duration;
    unsigned counts_per_step;
    long unsigned stop_count;
    std::vector<double> testSpeeds;
};

struct RampTestParams
{
    RampTestParams(double _test_ampl, double _ramp_duration, double _hold_duration, unsigned _F_s)
        : test_ampl(_test_ampl), ramp_duration(_ramp_duration), hold_duration(_hold_duration), F_s(_F_s) { }
    double test_ampl;
    unsigned F_s;
    double ramp_duration;
    double hold_duration;
    unsigned ramp_step_counts;
    unsigned hold_step_counts;
    long unsigned stop_count;
    std::vector<double> testCommand;
};

class SlewDriveTest
{
private:
    double motorACommand;
    double motorBCommand;
    // unsigned stopCount;
    SinTestParams *sinTestParamsPtr;
    FrictionTestParams *frictionTestParamsPtr;
    RampTestParams *rampTestParamsPtr;
    std::string logPathStr;

    KincoDriver *pDriveA;
    KincoDriver *pDriveB;
    TerminalInterface *terminal;
    bool connected;
    bool testConfigured;
    void updateCommands();
    void updateSinCommands();
    void updateFrictionCommands();
    void updateRampTestCommands();
    unsigned testCounter;
    bool testIsDone;

public:
    SlewDriveTest();
    void setupLogFile(const char *testIdStr);
    void setupTerminal(TerminalInterface *);
    void logDeltaTime();
    void collectFeedbackData();
    bool connectToDrivers(const char *);
    bool configureDrivers();
    void configureTest(SinTestParams *const paramsPtr);
    void configureTest(FrictionTestParams *const paramsPtr);
    void configureTest(RampTestParams *const paramsPtr);

    void shutdown();

    std::thread testUpdate()
    {
        return std::thread([=]
                           { updateCommands(); });
    }

    bool testComplete();
};