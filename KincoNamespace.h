#pragma once

namespace KINCO
{
    enum register_map_enum
    {
        CONTROL_WORD = 0x3100,
        STATUS_WORD = 0x3200,
        OPERATION_MODE = 0x3500,
        ABSOLUTE_RELATIVE_POSITIONING_CONTROL_SELECT = 0x0CF0,
        POS_ACTUAL = 0x3700,
        REAL_CURRENT = 0x3E00,
        STATUS_OF_INPUT_PORT = 0x6D00,
        REAL_SPEED = 0x3B00,
        INVERT_DIRECTION = 0x4700,
        TARGET_POSITION = 0x4000,
        PROFILE_SPEED = 0x4A00,
        TARGET_SPEED = 0x6F00,
        MAX_SPEED = 0x4900,
        PROFILE_ACC = 0x4B00,
        PROFILE_DEC = 0x4C00,
        TARGET_TORQUE = 0x3C00,
        GROUP_CURRENT_LOOP = 0x5880,
        MAXIMAL_CURRENT_COMMAND = 0x3D00,
        DIN_POS0 = 0x0C10,
        DIN_POS1 = 0x0C20,
        DIN_POS2 = 0x0C30,
        DIN_POS3 = 0x0C40,
        DIN_POS4 = 0x0D00,
        DIN_POS5 = 0x0D10,
        DIN_POS6 = 0x0D20,
        DIN_POS7 = 0x0D30,
        DIN_SPEED0 = 0x0C50,
        DIN_SPEED1 = 0x0C60,
        DIN_SPEED2 = 0x0C70,
        DIN_SPEED3 = 0x0C80,
        DIN_SPEED4 = 0x0D40,
        DIN_SPEED5 = 0x0D50,
        DIN_SPEED6 = 0x0D60,
        DIN_SPEED7 = 0x0D70,
        MAX_FOLLOWING_ERROR = 0x3800,
        TARGET_POS_WIND_OW = 0x3900,
        POSITION_WINDOW_TIME = 0x1990,
        TARGET_SPEED_WINDOW = 0x63A0,
        ZERO_SPEED_WINDOW = 0x0980,
        ZERO_SPEED_TIM_E = 0x6440,
        SOFT_POSITIVE_LIMIT = 0x4410,
        SOFT_NEGATIVE_LIMIT = 0x4420,
        LIMIT_FUNCTION = 0x0990,
        HOMING_METHOD = 0x4D00,
        HOMING_SPEED_SWITCH = 0x5010,
        HOMING_SPEED_ZERO = 0x5020,
        HOMING_ACCELERATION = 0x5200,
        HOMING_OFFSET = 0x4100,
        HOMING_OFFSET_MODE = 0x5050,
        KVP = 0x6310,
        KVI = 0x6320,
        KVI_OVER_32 = 0x6370,
        SPEED_FB_N = 0x6350,
        KPP = 0x6810,
        K_VELOCITY_FF = 0x6820,
        K_ACC_FF = 0x6830,
        POS_FILTER_N = 0x6850,
        DIN1_FUNCTION = 0x0830,
        DIN2_FUNCTION = 0x0840,
        DIN3_FUNCTION = 0x0850,
        DIN4_FUNCTION = 0x0860,
        DOUT1_FUNCTION = 0x08F0,
        DOUT2_FUNCTION = 0x0900,
        DIN_REAL = 0x08A0,
        DOUT_REAL = 0x0940,
        DIN_POLARITY = 0x0810,
        DOUT_POLARITY = 0x08D0,
        DIN_SIMULATE = 0x0820,
        DOUT_SIMULATE = 0x08E0,
        GEAR_FACTOR0 = 0x1910,
        GEAR_DIVIDER0 = 0x1920,
        PD_CW = 0x1930,
        GEAR_MASTER = 0x1940,
        GEAR_SLAVE = 0x1950,
        PD_FILTER = 0x1960,
        MASTER_SPEED = 0x19C0,
        SLAVE_SPEED = 0x19D0,
        STORE_DATA = 0x2910,
        STORE_MOTOR_DATA = 0x2930,
        ERROR_STATE = 0x1F00,
        QUICK_STOP_MODE = 0x3400,
        SHUTDOWN_ST_OP_MODE = 0x3410,
        DISABLE_STOP_MODE = 0x3420,
        HALT_MODE = 0x3430,
        FAULT_STOP_MODE = 0x3440,
        QUICK_STOP_DEC = 0x3300,
    };

    const uint16_t MODBUS_ERROR = 0x81;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    //////////////////////////////////////////////////////////////////////////////////////////////////
    struct STATUS_BITS
    {
        uint16_t READY_ON : 1;            // bit 0
        uint16_t SWITCH_ON : 1;           // bit 1
        uint16_t OPERATION_ENABLE : 1;    // bit 2
        uint16_t FAULT : 1;               // bit 3
        uint16_t VOLTAGE_ENABLE : 1;      // bit 4
        uint16_t QUICK_STOP : 1;          // bit 5
        uint16_t SWITCH_DISABLED : 1;     // bit 6
        uint16_t UNSPECIFIED : 1;         // bit 7
        uint16_t MANUFACTURE0 : 1;        // bit 8
        uint16_t REMOTE : 1;              // bit 9
        uint16_t TARGET_REACHED : 1;      // bit 10
        uint16_t INT_LIM_ACTIVE : 1;      // bit 11
        uint16_t SETPOINT_ACK : 1;        // bit 12
        uint16_t FOLLOWING_ERROR : 1;     // bit 13
        uint16_t COMMUNICATION_FOUND : 1; // bit 14
        uint16_t REFERENCE_FOUND : 1;     // bit 15
    };
    typedef union
    {
        struct STATUS_BITS BITS;
        uint16_t ALL;
    } StatusWord_t;


//////////////////////////////////////////////////////////////////////////////////////////////////
///
//////////////////////////////////////////////////////////////////////////////////////////////////

    enum motor_direction_enum
    {
        CCW_IS_POSITIVE = 0,
        CW_IS_POSITIVE = 1
    };

    enum motor_state_enum
    {
        // MOTOR_STATE_DISABLE = 0x02,
        POWER_OFF_MOTOR = 0x06, // must be off to enable and clear estop
        POWER_ON_MOTOR = 0x0F, // power on motor
        ESTOP_VOLTAGE_OFF = 0x0B,
        START_ABSOLUTE_1 = 0x2F,
        START_ABSOLUTE_2 = 0x3F,
        START_RELATIVE_1 = 0x4F,
        START_RELATIVE_2 = 0x5F,
        START_ABSOLUTE_TARGET_MOVING = 0x103F,
        HOME_POSITION_1 = 0x0F,
        HOME_POSITION_2 = 0x1F,
        CLEAR_SHOOTING = 0x80
    };

    enum motor_mode_enum
    {
        MOTOR_MODE_POSITION = 1,
        MOTOR_MODE_SPEED = 3,
        MOTOR_MODE_TORQUE = 4,
        MOTOR_MODE_HOME = 6
    };

    const uint32_t COUNTS_PER_REV = 10000;
    constexpr double MOTOR_MAX_SPEED_RPM = 3500;
    constexpr double MOTOR_MAX_SPEED_DPS = MOTOR_MAX_SPEED_RPM * 6; // 1 RPM = 6deg/s
}