#pragma once
#include <math.h>
#include <algorithm>

namespace AHRS{
extern float q0, q1, q2, q3; // 쿼터니언
extern void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
extern void calibrate_mahony_initial_attitude(float ax, float ay, float az, float mx, float my, float mz);
}