#pragma once
#ifndef __NN_PARAMETERS_DEF_H__
#define __NN_PARAMETERS_DEF_H__

#include <vector>

namespace NN {


static constexpr int N_ACT = 3;
static constexpr int N_STATE = 9;
static constexpr int N_GOAL = 3;
static constexpr int N_TASK = 8;
static constexpr int N_HIDDEN = 16;

static constexpr float AVEL_LIM = 15.700000;
static constexpr float VEL_LIM = 30.000000;
static constexpr float POS_LIM = 999999.900000;
static constexpr float AUTHORITY = 0.250000;

std::vector<float> OBS = {
