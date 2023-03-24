#pragma once

#include <inttypes.h>

template<typename T>
struct Vec3
{
    T x;
    T y;
    T z;
};

typedef Vec3<int16_t> Vec3i;

typedef Vec3<float> Vec3f;

typedef Vec3<double> Vec3d;