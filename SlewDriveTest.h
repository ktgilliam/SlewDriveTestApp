#include "KincoDriver.h"
#include <thread>
#include <vector>

enum
{
    SINUSOID_TEST,
    FRICTION_TEST,
    JSSG_2006
};

struct SinTestParams
{
    SinTestParams(double _amplitude, double _prd_sec, unsigned _num_prds, unsigned _F_s, uint8_t _type, uint8_t _mode)
        : amplitude(_amplitude),
          prd_sec(_prd_sec),
          num_prds(_num_prds),
          F_s(_F_s),
          type(_type),
          mode(_mode)
    {
        stop_count = (unsigned)(_num_prds * _prd_sec * _F_s);
    };
    enum
    {
        TORQUE_MODE,
        VELOCITY_MODE
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

class SlewDriveTest
{
private:
    double motorCommand;
    // unsigned stopCount;
    SinTestParams *sinTestParamsPtr;
    FrictionTestParams *frictionTestParamsPtr;

    uint8_t activeTestType;
    KincoDriver *pDriveA;
    KincoDriver *pDriveB;
    TerminalInterface *terminal;
    bool connected;
    bool testConfigured;
    void updateCommands();
    void updateSinCommands();
        void updateFrictionCommands();
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
    void shutdown();

    std::thread testUpdate()
    {
        return std::thread([=]
                           { updateCommands(); });
    }
    bool testComplete();
};