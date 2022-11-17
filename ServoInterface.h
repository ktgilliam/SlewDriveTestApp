#pragma once
#include "TerminalInterface.h"

class ServoInterface
{
protected:
    TerminalInterface *cli;
    bool terminalIsConnected{false};

public:
    virtual void connectTerminalInterface(TerminalInterface *_cli)
    {
        if (_cli != nullptr)
        {
            cli = _cli;
            terminalIsConnected = true;
        }
    }
    virtual void setDriverState(uint16_t) = 0;
    virtual void getDriverState() = 0;
    virtual void setControlMode(uint16_t) = 0;
    virtual void getControlMode() = 0;

    virtual void updatePositionCommand(double) = 0;
    virtual void updateVelocityCommand(double) = 0;
    virtual void updateTorqueCommand(double) = 0;

    virtual double getVelocityFeedback(bool updateConsole=false) = 0;
    virtual double getCurrentFeedback(bool updateConsole=false) = 0;
    virtual double getPositionFeedback(bool updateConsole=false) = 0;

    virtual void setupPersistentFields()
    {
        if (cli != nullptr)
        {
            cli->printPersistentFieldLabels();
        }
    }
    virtual void updateStatusField(unsigned fieldId, const std::string &val) = 0;
    virtual void readAndUpdateStatusField(unsigned fieldId) = 0;
    virtual void updateStatusFields() = 0;
};