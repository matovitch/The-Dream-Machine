#include "sse.h"

#define SIGNMASK (_mm_castsi128_ps(_mm_set1_epi32(0x80000000)))

__m128 sse_cross(__m128 a, __m128 b)
{
    __m128 shufa1 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 shufb1 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 shufa2 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2));
    __m128 shufb2 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));

    __m128 mult1 = _mm_mul_ps(shufa1, shufb1);
    __m128 mult2 = _mm_mul_ps(shufa2, shufb2);

    return _mm_sub_ps(mult1, mult2);
}

__m128 sse_negate(__m128 a)
{
    return _mm_xor_ps(a, SIGNMASK);
}

__m128 sse_dot4(__m128 a, __m128 b)
{
    __m128 mult = _mm_mul_ps(a, b);
    __m128 shuf1 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(0, 3, 2, 1));
    __m128 shuf2 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(1, 0, 3, 2));
    __m128 shuf3 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(2, 1, 0, 3));

    return _mm_add_ss(_mm_add_ss(mult, shuf1), _mm_add_ss(shuf2, shuf3));
}

__m128 sse_dot3(__m128 a, __m128 b)
{
    __m128 mult = _mm_mul_ps(a, b);
    __m128 shuf1 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 shuf2 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(3, 1, 0, 2));

    return _mm_add_ss(_mm_add_ss(mult, shuf1), shuf2);
}

__m128 sse_length2(__m128 a)
{
    __m128 mult = _mm_mul_ps(a, a);
    __m128 shuf1 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(3, 0, 2, 1));
    __m128 shuf2 = _mm_shuffle_ps(mult, mult, _MM_SHUFFLE(3, 1, 0, 2));

    return _mm_add_ss(_mm_add_ss(mult, shuf1), shuf2);
}

__m128 sse_normalize(__m128 a)
{
    __m128 len = _mm_sqrt_ss(sse_length2(a));
    return _mm_div_ps(a, _mm_shuffle_ps(len, len, _MM_SHUFFLE(0, 0, 0, 0)));
}

__m128 sse_recip(__m128 a)
{
    __m128 r = _mm_rcp_ss(a);
    __m128 r2 = _mm_add_ss(r, r);
    __m128 r3 = _mm_mul_ss(r, _mm_mul_ss(a, r));
    return _mm_sub_ss(r2, r3);
}

__m128 sse_recip4(__m128 a)
{
    __m128 r = _mm_rcp_ps(a);
    __m128 r2 = _mm_add_ps(r, r);
    __m128 r3 = _mm_mul_ps(r, _mm_mul_ps(a, r));
    return _mm_sub_ps(r2, r3);
}

__m128 sse_abs(__m128 a)
{
    return _mm_andnot_ps(SIGNMASK, a);
}

