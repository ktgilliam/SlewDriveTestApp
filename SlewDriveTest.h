#include "KincoDriver.h"
#include <thread>

enum
{
    SINUSOID_TEST,
    FRICTION_TEST,
    JSSG_2006
};

struct SinTestParams
{
    SinTestParams(double _amplitude, double _prd, unsigned _num_prds, unsigned _F_s, uint8_t _type, uint8_t _mode)
        : amplitude(_amplitude),
          prd_sec(_prd),
          num_prds(_num_prds),
          F_s(_F_s),
          type(_type),
          mode(_mode){};
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
};

class SlewDriveTest
{
private:
    double motorCommand;
    // unsigned stopCount;
    SinTestParams *sinTestParamsPtr;

    uint8_t activeTestType;
    KincoDriver *pDriveA;
    KincoDriver *pDriveB;
    TerminalInterface *terminal;
    bool connected;
    bool testConfigured;
    void updateSinCommands();

public:
    SlewDriveTest();
    void setupLogFile(const char *testIdStr);
    void setupTerminal(TerminalInterface *);
    void logDeltaTime();
    void collectFeedbackData();
    bool connectToDrivers(const char *);
    bool configureDrivers();
    void configureTest(SinTestParams *const paramsPtr);
    void shutdown();

    std::thread testUpdate()
    {
        return std::thread([=]
                           { updateSinCommands(); });
    }
};