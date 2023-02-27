#pragma once
#include "KincoNamespace.h"

#define UPDATE_RATE_HZ 10

#define TORQUE_SIN_MODE 0
#define VELOCITY_SIN_MODE 1
#define FRICTION_TEST_MODE 2
#define MIXED_CTRL_TEST_MODE 3


// UPDATE OP_MODE TO CHANGE THE TEST TYPE
#define OP_MODE TORQUE_SIN_MODE
//

#if OP_MODE == TORQUE_SIN_MODE
#define NUM_PERIODS 3
#define PRD_SEC 2
#define OP_MODE_STR "TRQ_"
const double TORQUE_AMPLITUDE = 0.5;

#elif OP_MODE == VELOCITY_SIN_MODE
#define NUM_PERIODS 3
#define PRD_SEC 10
#define OP_MODE_STR "VEL_"
// const double SPEED_AMPLITUDE = KINCO::MOTOR_MAX_SPEED_RPM;
const double SPEED_AMPLITUDE = KINCO::MOTOR_MAX_SPEED_RPM/2.0;

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

#elif OP_MODE == MIXED_CTRL_TEST_MODE
#define OP_MODE_STR "SDR_MYSTERY_"
const double MAX_SPEED = 100; // KINCO::MOTOR_MAX_SPEED_RPM;
const double RAMP_DURATION = 5.0;
const double HOLD_DURATION = 10.0;

#endif

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;

const double COULOMB_FRICTION_NM = 0.986301;
const double VISCOUS_FRICTION_NM_PER_RPM = 0.006314798000193;


#define DRIVER_A_ID 3
#define DRIVER_B_ID 4