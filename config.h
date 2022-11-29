#pragma once
#include "KincoNamespace.h"

#define TORQUE_MODE 0
#define VELOCITY_MODE 1

#define OP_MODE VELOCITY_MODE

#if OP_MODE == TORQUE_MODE
#define PRD_SEC 30
#define OP_MODE_STR "TRQ_"
#else
#define PRD_SEC 30
#define OP_MODE_STR "VEL_"
#endif

#define NUM_PERIODS 3
#define UPDATE_RATE_HZ 10

const int F_s = UPDATE_RATE_HZ;
constexpr double T_s = 1 / (double)F_s;

constexpr int stop_count = NUM_PERIODS * PRD_SEC * F_s;
const unsigned int period_sec = PRD_SEC;
constexpr double f_0 = 1 / (double)period_sec;


const double torqueAmplitude = 1.0;
const double speedAmplitude = KINCO::MOTOR_MAX_SPEED_RPM;

const bool TERMINAL_DRIVE_A = true;
const bool TERMINAL_DRIVE_B = true;