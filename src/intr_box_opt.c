#include "intr.h"
#include <xmmintrin.h>

void intr_packet_box(const ray_packet* packet,
                     const float* box_min,
                     const float* box_max,
                     const float* prev_tmin,
                     int* hit)
{
    const __m128 bmin = _mm_load_ps(box_min);
    const __m128 bmax = _mm_load_ps(box_max);

    __m128 rorg = _mm_load_ps(packet->org);
    __m128 rinv = _mm_load_ps(packet->inv_dir);
    const __m128 t00 = _mm_mul_ps(_mm_sub_ps(bmin, rorg), rinv);
    const __m128 t10 = _mm_mul_ps(_mm_sub_ps(bmax, rorg), rinv);

    rorg = _mm_load_ps(packet->org + 4);
    rinv = _mm_load_ps(packet->inv_dir + 4);
    const __m128 t01 = _mm_mul_ps(_mm_sub_ps(bmin, rorg), rinv);
    const __m128 t11 = _mm_mul_ps(_mm_sub_ps(bmax, rorg), rinv);

    rorg = _mm_load_ps(packet->org + 8);
    rinv = _mm_load_ps(packet->inv_dir + 8);
    const __m128 t02 = _mm_mul_ps(_mm_sub_ps(bmin, rorg), rinv);
    const __m128 t12 = _mm_mul_ps(_mm_sub_ps(bmax, rorg), rinv);

    rorg = _mm_load_ps(packet->org + 12);
    rinv = _mm_load_ps(packet->inv_dir + 12);
    const __m128 t03 = _mm_mul_ps(_mm_sub_ps(bmin, rorg), rinv);
    const __m128 t13 = _mm_mul_ps(_mm_sub_ps(bmax, rorg), rinv);

    __m128 tmin0 = _mm_min_ps(t00, t10);
    __m128 tmax0 = _mm_max_ps(t00, t10);

    __m128 tmin1 = _mm_min_ps(t01, t11);
    __m128 tmax1 = _mm_max_ps(t01, t11);

    __m128 tmin2 = _mm_min_ps(t02, t12);
    __m128 tmax2 = _mm_max_ps(t02, t12);

    __m128 tmin3 = _mm_min_ps(t03, t13);
    __m128 tmax3 = _mm_max_ps(t03, t13);

    /* Min, max over (x, y) and (y, z) */
    tmin0 = _mm_max_ps(tmin0, _mm_shuffle_ps(tmin0, tmin0, _MM_SHUFFLE(3, 0, 2, 1)));
    tmax0 = _mm_min_ps(tmax0, _mm_shuffle_ps(tmax0, tmax0, _MM_SHUFFLE(3, 0, 2, 1)));

    tmin1 = _mm_max_ps(tmin1, _mm_shuffle_ps(tmin1, tmin1, _MM_SHUFFLE(3, 0, 2, 1)));
    tmax1 = _mm_min_ps(tmax1, _mm_shuffle_ps(tmax1, tmax1, _MM_SHUFFLE(3, 0, 2, 1)));

    tmin2 = _mm_max_ps(tmin2, _mm_shuffle_ps(tmin2, tmin2, _MM_SHUFFLE(3, 0, 2, 1)));
    tmax2 = _mm_min_ps(tmax2, _mm_shuffle_ps(tmax2, tmax2, _MM_SHUFFLE(3, 0, 2, 1)));

    tmin3 = _mm_max_ps(tmin3, _mm_shuffle_ps(tmin3, tmin3, _MM_SHUFFLE(3, 0, 2, 1)));
    tmax3 = _mm_min_ps(tmax3, _mm_shuffle_ps(tmax3, tmax3, _MM_SHUFFLE(3, 0, 2, 1)));

    /* Min, max over (x, y, z) */
    tmin0 = _mm_max_ps(tmin0, _mm_shuffle_ps(tmin0, tmin0, _MM_SHUFFLE(3, 2, 0, 1)));
    tmax0 = _mm_min_ps(tmax0, _mm_shuffle_ps(tmax0, tmax0, _MM_SHUFFLE(3, 2, 0, 1)));

    tmin1 = _mm_max_ps(tmin1, _mm_shuffle_ps(tmin1, tmin1, _MM_SHUFFLE(3, 2, 0, 1)));
    tmax1 = _mm_min_ps(tmax1, _mm_shuffle_ps(tmax1, tmax1, _MM_SHUFFLE(3, 2, 0, 1)));

    tmin2 = _mm_max_ps(tmin2, _mm_shuffle_ps(tmin2, tmin2, _MM_SHUFFLE(3, 2, 0, 1)));
    tmax2 = _mm_min_ps(tmax2, _mm_shuffle_ps(tmax2, tmax2, _MM_SHUFFLE(3, 2, 0, 1)));

    tmin3 = _mm_max_ps(tmin3, _mm_shuffle_ps(tmin3, tmin3, _MM_SHUFFLE(3, 2, 0, 1)));
    tmax3 = _mm_min_ps(tmax3, _mm_shuffle_ps(tmax3, tmax3, _MM_SHUFFLE(3, 2, 0, 1)));

    /* Build the final tmin and tmax values */
    const __m128 tmin01 = _mm_unpacklo_ps(tmin0, tmin1);
    const __m128 tmax01 = _mm_unpacklo_ps(tmax0, tmax1);

    const __m128 tmin23 = _mm_unpacklo_ps(tmin2, tmin3);
    const __m128 tmax23 = _mm_unpacklo_ps(tmax2, tmax3);

    const __m128 tmin = _mm_shuffle_ps(tmin01, tmin23, _MM_SHUFFLE(1, 0, 1, 0));
    const __m128 tmax = _mm_shuffle_ps(tmax01, tmax23, _MM_SHUFFLE(1, 0, 1, 0));
    
    /* ray intersects iff tmax[0, 1, 2, 3] >= tmin[0, 1, 2, 3] && tmax[0, 1, 2, 3] >= 0 */
    __m128 mask = _mm_and_ps(_mm_cmpge_ps(tmax, tmin),
                             _mm_cmpge_ps(tmax, _mm_setzero_ps()));
    /* We also cull intersections below the given threshold */
    mask = _mm_and_ps(mask, _mm_cmple_ps(tmin, _mm_load_ps(prev_tmin)));

    _mm_store_ps((float*)hit, mask);
}
