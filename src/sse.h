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

#ifndef DREAM_SSE_H
#define DREAM_SSE_H

#include <xmmintrin.h>
#include <emmintrin.h>

/**
 * \file sse.h
 * \brief SSE2 routines
 * \author Arsène Pérard-Gayot, Camille Brugel
 * \date 10 juin 2013
 */

/* All those routines use SSE2, you may want to replace them
 * with SSE3 or SSE4 intrinsics.
 */

/** Cross product, fourth component is zero */
__m128 sse_cross(__m128 a, __m128 b);
/** Vector negate operation */
__m128 sse_negate(__m128 a);
/** 4 components dot product, result in low word */
__m128 sse_dot4(__m128 a, __m128 b);
/** 3 components dot product, result in low word */
__m128 sse_dot3(__m128 a, __m128 b);
/** Square length, result in low word */
__m128 sse_length2(__m128 a);
/** Vector normalize operation - precise */
__m128 sse_normalize(__m128 a);
/** 1 component reciprocal, result in low word - precise */
__m128 sse_recip(__m128 a);
/** 4 components reciprocal, result in low word - precise */
__m128 sse_recip4(__m128 a);
/** Vector absolute value */
__m128 sse_abs(__m128 a);

#endif // DREAM_SSE_H
