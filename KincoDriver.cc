#include "KincoDriver.h"
#include <cstring>
#include <cinttypes>
#include <string>
#include <stdexcept>

#include "KinkoNamespace.h"
#include "BitFieldUtil.h"

#define ERR_BUFF_SIZE 40

uint16_t KincoDriver::numDrivers = 0;

double convertSpeedIUtoRPM(int32_t speed_units);
double convertCurrIUtoAmp(int32_t current_units);
int32_t convertSpeedRPMtoIU(int16_t speed_rpm);

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
KincoDriver::KincoDriver(int16_t driverId)
    : driverNodeId(driverId)
{
    numDrivers++;
    ctx = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::connectRTU(const char *device, int baud, char parity, int data_bit, int stop_bit)
{
    ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    if (ctx == NULL)
    {
        throw std::runtime_error("Unable to create the libmodbus context\n");
    }

    modbus_set_slave(ctx, driverNodeId);

    if (modbus_connect(ctx) == -1)
    {
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        throw std::runtime_error(errBuff);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
T KincoDriver::readDriverRegister(uint16_t modBusAddr)
{
    if (ctx == NULL)
        throw std::runtime_error("Modbus context not established.");

    uint16_t result_code = 0;
    constexpr uint16_t numWords = sizeof(T) / sizeof(uint16_t);
    ConversionBuffer<T> rxBuff;

    result_code = modbus_read_registers(ctx, modBusAddr, numWords, rxBuff.U16_PARTS);
    if (result_code == -1)
    {
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "%s\n", modbus_strerror(errno));
        throw std::runtime_error(errBuff);
    }
    return static_cast<T>(rxBuff.WHOLE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
uint16_t KincoDriver::writeDriverRegisters(uint16_t modBusAddr, T reg_value)
{
    if (ctx == NULL)
        throw std::runtime_error("Modbus context not established.");
    uint16_t result_code = 0;
    uint16_t numWords = sizeof(T) / sizeof(uint16_t);

    ConversionBuffer<T> txBuff;

    if (numWords == 1)
    {

        result_code = modbus_write_register(ctx, modBusAddr, reg_value);
    }
    else
    {
        txBuff.WHOLE = reg_value;
        result_code = modbus_write_registers(ctx, modBusAddr, numWords, txBuff.U16_PARTS);
    }
    if (result_code == -1)
    {
        char errBuff[ERR_BUFF_SIZE];
        sprintf(errBuff, "%s\n", modbus_strerror(errno));
        throw std::runtime_error(errBuff);
    }
    return result_code;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setDriverState(uint16_t motor_state)
{
    writeDriverRegisters<uint16_t>(KINKO::CONTROL_WORD, motor_state);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setControlMode(uint16_t motor_mode)
{
    writeDriverRegisters<uint16_t>(KINKO::OPERATION_MODE, motor_mode);
    if (cli != nullptr)
    {
        // enum motor_mode_enum modeSwitch = (enum motor_mode_enum) motor_mode;
        switch (motor_mode)
        {
        case KINKO::MOTOR_MODE_POSITION:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "POSITION MODE");
            break;
        case KINKO::MOTOR_MODE_SPEED:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "RATE MODE");
            break;
        case KINKO::MOTOR_MODE_TORQUE:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "TORQUE MODE");
            break;
        case KINKO::MOTOR_MODE_HOME:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "HOMING MODE");
            break;
        default:
            updateStatusField(KINKO::DRIVER_MODE_ROW, "MODE UNKNOWN");
            break;
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updatePositionCommand(double posn_setpoint)
{
    writeDriverRegisters<int32_t>(KINKO::TARGET_POSITION, posn_setpoint);

    if (cli != nullptr)
    {
        cli->updatePersistentField(KINKO::CMD_ROW, posn_setpoint);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateVelocityCommand(double velocity_setpoint)
{
    int32_t target_speed_value = convertSpeedRPMtoIU(velocity_setpoint);
    // int32_t target_speed_value = 0x;
    writeDriverRegisters<int32_t>(KINKO::TARGET_SPEED, target_speed_value);
    if (cli != nullptr)
    {
        cli->updatePersistentField(KINKO::CMD_ROW, velocity_setpoint);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::updateTorqueCommand(double torque_setpoint)
{
    writeDriverRegisters<int32_t>(KINKO::TARGET_TORQUE, torque_setpoint);

    if (cli != nullptr)
    {
        cli->updatePersistentField(KINKO::CMD_ROW, torque_setpoint);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getVelocityFeedback(bool updateConsole)
{
    static double tmpv = 0.0;
    tmpv += 0.001;
    // int32_t real_speed_units = readDriverRegister(KINKO::REAL_SPEED);
    auto real_speed_units = readDriverRegister<int32_t>(KINKO::REAL_SPEED);
    auto real_speed_rpm = convertSpeedIUtoRPM(real_speed_units);
    if(updateConsole && cli != nullptr)
    {
        real_speed_rpm = tmpv;
        cli->updatePersistentField(KINKO::VEL_FB_ROW, real_speed_rpm);
    }
    return real_speed_rpm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getCurrentFeedback(bool updateConsole)
{
    auto real_current_units = readDriverRegister<int32_t>(KINKO::REAL_CURRENT);
    int32_t real_current_amps = convertCurrIUtoAmp(real_current_units);
    if(updateConsole && cli != nullptr)
    {
        cli->updatePersistentField(KINKO::TRQ_FB_ROW, real_current_amps);
    }
    return real_current_amps;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double KincoDriver::getPositionFeedback(bool updateConsole)
{
    auto encoder_counts = readDriverRegister<int32_t>(KINKO::POS_ACTUAL);
    // TODO: Convert to eng units
    if(updateConsole && cli != nullptr)
    {
        cli->updatePersistentField(KINKO::POSN_FB_ROW, encoder_counts);
    }
    return encoder_counts;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::connectTerminalInterface(TerminalInterface *_cli)
{
    ServoInterface::connectTerminalInterface(_cli);
    if (terminalIsConnected)
        setupPersistentFields();
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
void KincoDriver::setupPersistentFields()
{
    cli->addPersistentField("Driver Status", KINKO::DRIVER_STATUS_ROW);
    cli->addPersistentField("Driver Mode", KINKO::DRIVER_MODE_ROW);
    cli->addPersistentField("Command", KINKO::CMD_ROW);
    cli->addPersistentField("Position FB [deg]", KINKO::POSN_FB_ROW);
    cli->addPersistentField("Velocity FB [RPM]", KINKO::VEL_FB_ROW);
    cli->addPersistentField("Torque FB [%]", KINKO::TRQ_FB_ROW);

    ServoInterface::setupPersistentFields();
    updateStatusFields();
}
void KincoDriver::updateStatusField(unsigned fieldId, const std::string &val)
{
    if (cli != nullptr)
    {
        cli->updatePersistentField(fieldId, val);
    }
}
void KincoDriver::readAndUpdateStatusField(unsigned fieldId)
{
    enum KINKO::KincoPersistentFields fieldSwitch = (enum KINKO::KincoPersistentFields)fieldId;

    switch (fieldSwitch)
    {
    case KINKO::DRIVER_MODE_ROW:
        // read driver mode
        cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    case KINKO::DRIVER_STATUS_ROW:
        // read driver status
        cli->updatePersistentField(fieldSwitch, "INIT");
        break;
    }
}

void KincoDriver::updateStatusFields()
{
    cli->updatePersistentField(KINKO::DRIVER_MODE_ROW, "INIT");
    cli->updatePersistentField(KINKO::DRIVER_STATUS_ROW, "INIT");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertSpeedIUtoRPM(int32_t speed_units)
{
    return (((double)speed_units * 1875) / (512 * 10000));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
double convertCurrIUtoAmp(int32_t current_units)
{
    return (double)current_units / KINKO::amps2counts * 100.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////
int32_t convertSpeedRPMtoIU(int16_t speed_rpm)
{
    int32_t speed_units = 0;
    double holder = 0;
    holder = (double)speed_rpm * KINKO::rpm2cps;
    speed_units = (int32_t)holder;
    return speed_units;
}
