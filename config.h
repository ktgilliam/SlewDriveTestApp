#pragma once
#include "KincoNamespace.h"

#define XSTR(x) STR(x)
#define STR(x) #x

#define SIDEREAL_RATE_MTR_RPM 6.25
#define SR_MULT 10

#define UPDATE_RATE_HZ 30


#define TORQUE_SIN_TEST_MODE 0
#define VELOCITY_SIN_TEST_MODE 1
#define FRICTION_TEST_MODE 2
#define RAMP_TEST_MODE 3
#define JSSG_2006_TEST_MODE 4
// #define SIN_MODE 5

////////////////////////
#define OP_MODE RAMP_TEST_MODE
//////////////////////////

#define MOTOR_TORQUE_MODE 1
#define MOTOR_VELOCITY_MODE 2
#define MOTOR_POSITION_MODE 3

#if OP_MODE == TORQUE_SIN_TEST_MODE
#define NUM_PERIODS 3
#define PRD_SEC 3
#define OP_MODE_STR "TRQ_"
const double TORQUE_AMPLITUDE = 1.0;

#elif OP_MODE == VELOCITY_SIN_TEST_MODE
#define MOTOR_B_MODE MOTOR_VELOCITY_MODE
// #define MOTOR_B_MODE MOTOR_TORQUE_MODE
#define NUM_PERIODS 3
#define PRD_SEC 30
#define OP_MODE_STR "VEL_"
const double SPEED_AMPLITUDE = KINCO::MOTOR_MAX_SPEED_RPM*0.2;

#elif OP_MODE == FRICTION_TEST_MODE
// #define OP_MODE_STR "FRICTION_"
// #define OP_MODE_STR "MTR_FRCTN_"
// #define OP_MODE_STR "GBX_FRCTN_"
#define OP_MODE_STR "SDR_FRCTN_"
const unsigned STEPS_PER_SIDE = 20;
const double STEP_DURATION = 2.0;
// constexpr double TEST_DURATION = STEP_DURATION * STEPS_PER_SIDE * 2;
// constexpr unsigned stop_count = (unsigned)TEST_DURATION * UPDATE_RATE_HZ + 1;
const double MAX_SPEED = KINCO::MOTOR_MAX_SPEED_RPM;

#elif OP_MODE == RAMP_TEST_MODE
#define BOTH_DIRECTIONS true
// #define MOTOR_B_MODE MOTOR_VELOCITY_MODE
#define MOTOR_B_MODE MOTOR_TORQUE_MODE
#define MIXED_MODE (MOTOR_B_MODE != MOTOR_VELOCITY_MODE)
#if MIXED_MODE
#define OP_MODE_STR "SDR_RAMP_VT_"
#else
#define OP_MODE_STR "SDR_RAMP_VV_"
#endif

// #define MIXED_MODE_SPEED_MULT 1
// const double MAX_SPEED = KINCO::MOTOR_MAX_SPEED_RPM*0.2*MIXED_MODE_SPEED_MULT;
// const double RAMP_DURATION = 3.0/MIXED_MODE_SPEED_MULT;
// const double HOLD_DURATION = 5.0/MIXED_MODE_SPEED_MULT;

const double MAX_SPEED = SIDEREAL_RATE_MTR_RPM*1;
const double RAMP_DURATION = 2.0;
const double HOLD_DURATION = 10.0;

#endif

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;

const double COULOMB_FRICTION_NM = 0.986301;
const double VISCOUS_FRICTION_NM_PER_RPM = 0.0006314798000193;


#define AZIMUTH_AXIS 0
#define ELEVATION_AXIS 1

#define ACTIVE_AXIS ELEVATION_AXIS


#if ACTIVE_AXIS == AZIMUTH_AXIS
#define DRIVER_A_ID 1
#define DRIVER_B_ID 2
#elif ACTIVE_AXIS == ELEVATION_AXIS
#define DRIVER_A_ID 3
#define DRIVER_B_ID 4
#endif

#define CONSOLE_DOWNSAMPLE 10