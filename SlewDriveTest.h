#include "KincoDriver.h"
#include <thread>

class SlewDriveTest
{
private:
    KincoDriver *pDriveA;
    KincoDriver *pDriveB;
    TerminalInterface *terminal;
    bool connected;
    void updateCommands();
public:
    SlewDriveTest();
    void setupLogFile();
    void setupTerminal(TerminalInterface *);
    void logDeltaTime();
    void collectFeedbackData();
    bool connectToDrivers(const char*);
    bool configureDriversForTest();
    void shutdown();

    std::thread testUpdate(){ return std::thread([=] { updateCommands(); });}
};