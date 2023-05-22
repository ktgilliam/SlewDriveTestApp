#pragma once
#include "KincoNamespace.h"

#define XSTR(x) STR(x)
#define STR(x) #x

#define SIDEREAL_RATE_MTR_RPM 6.25
#define SR_MULT 10

#define UPDATE_RATE_HZ 30
#define UPDATE_PERIOD_SEC (1.0/double(UPDATE_RATE_HZ))
#define CONSOLE_DOWNSAMPLE UPDATE_RATE_HZ

#define TORQUE_SIN_TEST_MODE 0
#define VELOCITY_SIN_TEST_MODE 1
#define FRICTION_TEST_MODE 2
#define RAMP_TEST_MODE 3
// #define SIN_MODE 5

////////////////// //////
#define OP_MODE TORQUE_SIN_TEST_MODE
//////////////////////////

const double MAX_SPEED = KINCO::MOTOR_MAX_SPEED_RPM;

#define MOTOR_TORQUE_MODE 1
#define MOTOR_VELOCITY_MODE 2
#define MOTOR_POSITION_MODE 3

#if OP_MODE == TORQUE_SIN_TEST_MODE

#define NUM_PERIODS 4
#define PRD_SEC     40
#define OP_MODE_STR "TRQ_"
const double TEST_AMPL = 1.0;

#elif OP_MODE == VELOCITY_SIN_TEST_MODE
#define MOTOR_B_MODE MOTOR_VELOCITY_MODE
// #define MOTOR_B_MODE MOTOR_TORQUE_MODE
#define OP_MODE_STR "VEL_"
#define NUM_PERIODS 3
#define PRD_SEC 15
const double SPEED_AMPLITUDE = KINCO::MOTOR_MAX_SPEED_RPM;
//////////// Warmup settingsconst double MAX_SPEED = KINCO::MOTOR_MAX_SPEED_RPM;


#elif OP_MODE == FRICTION_TEST_MODE
// #define OP_MODE_STR "FRICTION_"
// #define OP_MODE_STR "MTR_FRCTN_"
// #define OP_MODE_STR "GBX_FRCTN_"
#define OP_MODE_STR "SDR_FRCTN_"
const unsigned STEPS_PER_SIDE = 10;
const double STEP_DURATION = 2.0;
// constexpr double TEST_DURATION = STEP_DURATION * STEPS_PER_SIDE * 2;
// constexpr unsigned stop_count = (unsigned)TEST_DURATION * UPDATE_RATE_HZ + 1;
const double TEST_AMPL = MAX_SPEED;

#elif OP_MODE == RAMP_TEST_MODE
#ifdef JSSG_2006_TEST_MODE
#   define OP_MODE_STR "JSSG_2006_"
#   define BOTH_DIRECTIONS true
#   define MOTOR_A_MODE MOTOR_TORQUE_MODE
#   define MOTOR_B_MODE MOTOR_TORQUE_MODE
const double RAMP_DURATION = 5.0;
const double HOLD_DURATION = 2.0;
const double TEST_AMPL = KINCO::PEAK_TORQUE_NM*-1.5;
#else
#   define BOTH_DIRECTIONS false
#   define MOTOR_A_MODE MOTOR_VELOCITY_MODE
#   define MOTOR_B_MODE MOTOR_VELOCITY_MODE
// #define MOTOR_B_MODE MOTOR_TORQUE_MODE
#   define MIXED_MODE (MOTOR_B_MODE != MOTOR_VELOCITY_MODE)
#   if MIXED_MODE
#       define OP_MODE_STR "SDR_RAMP_VT_"
#       else
#       define OP_MODE_STR "SDR_RAMP_VV_"
#       ifdef DEBUG_MODE
const double TEST_AMPL = MAX_SPEED;
const double RAMP_DURATION = 5.0;
const double HOLD_DURATION = 10.0;
#       else
// const double TEST_AMPL = SIDEREAL_RATE_MTR_RPM*-1;
const double TEST_AMPL = -150;
const double RAMP_DURATION = 5.0;
const double HOLD_DURATION = 1200.0;
#       endif
#   endif

#   endif
#endif

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;

const double COULOMB_FRICTION_NM = 0.986301;
const double VISCOUS_FRICTION_NM_PER_RPM = 0.0006314798000193;


#define AZIMUTH_AXIS 0
#define ELEVATION_AXIS 1

#define ACTIVE_AXIS AZIMUTH_AXIS


#if ACTIVE_AXIS == AZIMUTH_AXIS
#define DRIVER_A_ID 1
#define DRIVER_B_ID 2
#elif ACTIVE_AXIS == ELEVATION_AXIS
#define DRIVER_A_ID 3
#define DRIVER_B_ID 4
#endif


