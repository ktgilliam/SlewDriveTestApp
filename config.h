#pragma once
#include "KincoNamespace.h"

#define TORQUE_SIN_MODE 0
#define VELOCITY_SIN_MODE 1

#define OP_MODE VELOCITY_SIN_MODE

#if OP_MODE == TORQUE_SIN_MODE
#define PRD_SEC 10
#define OP_MODE_STR "TRQ_"
#elif OP_MODE == VELOCITY_SIN_MODE
#define PRD_SEC 10
#define OP_MODE_STR "VEL_"
#endif

#define NUM_PERIODS 1
#define UPDATE_RATE_HZ 10

const int F_s = UPDATE_RATE_HZ;
constexpr int stop_count = NUM_PERIODS * PRD_SEC * F_s;

const double TORQUE_AMPLITUDE = 1.0;
const double SPEED_AMPLITUDE = KINCO::MOTOR_MAX_SPEED_RPM;

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;