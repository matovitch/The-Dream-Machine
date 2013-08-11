#include "linalg.h"
#include <math.h>

void vector3_copy(const vector3 a, vector3 b)
{
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
}

void vector3_set(float a, float b, float c, vector3 d)
{
    d[0] = a;
    d[1] = b;
    d[2] = c;
}

void vector3_set_all(float a, vector3 b)
{
    b[0] = a;
    b[1] = a;
    b[2] = a;
}

void vector3_negate(const vector3 a, vector3 b)
{
    b[0] = -a[0];
    b[1] = -a[1];
    b[2] = -a[2];
}

void vector3_add(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

void vector3_subtract(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

void vector3_max(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = (a[0] > b[0]) ? a[0] : b[0];
    c[1] = (a[1] > b[1]) ? a[1] : b[1];
    c[2] = (a[2] > b[2]) ? a[2] : b[2];
}

void vector3_min(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = (a[0] < b[0]) ? a[0] : b[0];
    c[1] = (a[1] < b[1]) ? a[1] : b[1];
    c[2] = (a[2] < b[2]) ? a[2] : b[2];
}

void vector3_scale(const vector3 a, float b, vector3 c)
{
    c[0] = a[0] * b;
    c[1] = a[1] * b;
    c[2] = a[2] * b;
}

void vector3_scale3(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = a[0] * b[0];
    c[1] = a[1] * b[1];
    c[2] = a[2] * b[2];
}

void vector3_cross(const vector3 a, const vector3 b, vector3 c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

float vector3_dot(const vector3 a, const vector3 b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float vector3_length2(const vector3 a)
{
    return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}

float vector3_length(const vector3 a)
{
    return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

void vector4_copy(const vector4 a, vector4 b)
{
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
    b[3] = a[3];
}

void vector4_add(const vector4 a, const vector4 b, vector4 c)
{
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
    c[3] = a[3] + b[3];
}

void vector4_scale(const vector4 a, float b, vector4 c)
{
    c[0] = a[0] * b;
    c[1] = a[1] * b;
    c[2] = a[2] * b;
    c[3] = a[3] * b;
}

void vector4_scale4(const vector4 a, const vector4 b, vector4 c)
{
    c[0] = a[0] * b[0];
    c[1] = a[1] * b[1];
    c[2] = a[2] * b[2];
    c[3] = a[3] * b[3];
}

void quat_mult(const quat a, const quat b, quat c)
{
    c[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    c[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    c[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    c[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

void quat_conj(const quat a, quat b)
{
    b[0] = -a[0];
    b[1] = -a[1];
    b[2] = -a[2];
    b[3] = a[3];
}

