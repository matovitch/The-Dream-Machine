/** (C) 2013-2014 MadMann's Company
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

 /**
 * \file linalg.h
 * \brief Linear Algebra
 * \author Arsène Pérard-Gayot, Camille Brugel
 * \date 10 juin 2013
 */

#ifndef DREAM_LINALG_H
#define DREAM_LINALG_H

/* These functions are provided for convenience only. They should not
 * be used in production code, since an implementation using SSE intrinsics
 * will generally be faster.
 */

typedef float vector3[3];
typedef float vector4[4];
typedef float quat[4];
typedef float matrix3[9];
typedef float matrix4[16];

void vector3_copy(const vector3 a, vector3 b);
void vector3_set(float a, float b, float c, vector3 d);
void vector3_set_all(float a, vector3 b);
void vector3_negate(const vector3 a, vector3 b);
void vector3_add(const vector3 a, const vector3 b, vector3 c);
void vector3_subtract(const vector3 a, const vector3 b, vector3 c);
void vector3_max(const vector3 a, const vector3 b, vector3 c);
void vector3_min(const vector3 a, const vector3 b, vector3 c);
void vector3_scale(const vector3 a, float b, vector3 c);
void vector3_scale3(const vector3 a, const vector3 b, vector3 c);
void vector3_cross(const vector3 a, const vector3 b, vector3 c);
float vector3_dot(const vector3 a, const vector3 b);
float vector3_length2(const vector3 a);
float vector3_length(const vector3 a);

void vector4_copy(const vector4 a, vector4 b);
void vector4_add(const vector4 a, const vector4 b, vector4 c);
void vector4_scale(const vector4 a, float b, vector4 c);
void vector4_scale4(const vector4 a, const vector4 b, vector4 c);

void quat_mult(const quat a, const quat b, quat c);
void quat_conj(const quat a, quat b); 

#endif // DREAM_LINALG_H
