#include "intr.h"
#include "mem.h"
#include <xmmintrin.h>

int intr_frustum_box(const float* frustum, const float* box_min, const float* box_max)
{
    const __m128 min = _mm_load_ps(box_min);
    const __m128 max = _mm_load_ps(box_max);

    for (int i = 0; i < 6; i++) {
        const __m128 plane = _mm_load_ps(frustum + 4 * i);

        const __m128 mask = _mm_cmplt_ps(plane, _mm_setzero_ps());
        const __m128 n = _mm_or_ps(_mm_and_ps(mask, max),
                                   _mm_andnot_ps(mask, min));

        const __m128 d = _mm_mul_ps(n, plane);
        const __m128 d0 = _mm_shuffle_ps(d, d, _MM_SHUFFLE(2, 1, 0, 3));
        const __m128 d1 = _mm_shuffle_ps(d, d, _MM_SHUFFLE(1, 0, 3, 2));
        const __m128 d2 = _mm_shuffle_ps(d, d, _MM_SHUFFLE(0, 3, 2, 1));
        const __m128 dot = _mm_add_ss(_mm_add_ss(d0, d), _mm_add_ss(d1, d2));

        const __m128 ret = _mm_cmpgt_ss(dot, _mm_setzero_ps());
        
        float reti;
        _mm_store_ss(&reti, ret);
        if (reti != 0)
            return 0;
    }
    
    return 1;
}
