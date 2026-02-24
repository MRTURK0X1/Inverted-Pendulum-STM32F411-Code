#pragma once

#include "main.h"

#define PI        3.14159265359f
#define TWO_PI    (2.0f * PI)
#define HALF_PI   (PI / 2.0f)

float wrap_to_pi(float x);

float utility_cos(float x);

float utility_sin(float x);

float v_12_saturate(float value);

int16_t v_12_to_arr(float v_12);























