#include "intr.h"
#include "linalg.h"
#include "sse.h"
#ifdef __SSE4_1__
#include <smmintrin.h>
#endif

void intr_packet_tri(const ray_packet* packet,
                     const precomp_tri* precomp,
                     packet_hit* hit)
{
    hit->intr[0] = 0;
    hit->intr[1] = 0;
    hit->intr[2] = 0;
    hit->intr[3] = 0;

    const __m128 n = _mm_load_ps(precomp->n);
    const __m128 u = _mm_load_ps(precomp->u);
    const __m128 v = _mm_load_ps(precomp->v);

    for (int i = 0; i < 4; i++) {
        const __m128 o = _mm_load_ps(packet->org + 4 * i);
        const __m128 d = _mm_load_ps(packet->dir + 4 * i);

#ifdef __SSE4_1__
        const __m128 det = _mm_dp_ps(n, d, 0x7f);
        const __m128 dett = _mm_dp_ps(_mm_mul_ps(_mm_set_ps(1, -1, -1, -1), n),
                                o, 0xff);
#else
        __m128 det = sse_dot3(n, d);
        __m128 dett = sse_dot4(_mm_mul_ps(_mm_set_ps(1, -1, -1, -1), n), o);
        det = _mm_shuffle_ps(det, det, _MM_SHUFFLE(0, 0, 0, 0));
        dett = _mm_shuffle_ps(dett, dett, _MM_SHUFFLE(0, 0, 0, 0));
#endif
        const __m128 oldt = _mm_load_ss(&hit->t[i]);

        if((_mm_movemask_ps(_mm_xor_ps(dett,
            _mm_sub_ss(_mm_mul_ss(oldt, det), dett))) & 1) == 0) {

            /* d[4] must be zero /!\ */
            const __m128 detp = _mm_add_ps(_mm_mul_ps(o, det),
                                           _mm_mul_ps(dett, d));
#ifdef __SSE4_1__
            const __m128 detu = _mm_dp_ps(detp, u, 0xf1);

#else
            const __m128 detu = sse_dot4(detp, u);
#endif

            if((_mm_movemask_ps(_mm_xor_ps(detu,
                _mm_sub_ss(det, detu))) & 1) == 0) {

#ifdef __SSE4_1__
                const __m128 detv = _mm_dp_ps(detp, v, 0xf1);
#else
                const __m128 detv = sse_dot4(detp, v);
#endif

                if((_mm_movemask_ps(_mm_xor_ps(detv,
                    _mm_sub_ss(det, _mm_add_ss(detu, detv)))) & 1) == 0) {
                    const __m128 inv_det = sse_recip(det);
                    _mm_store_ss(&hit->t[i], _mm_mul_ss(dett, inv_det));
                    _mm_store_ss(&hit->u[i], _mm_mul_ss(detu, inv_det));
                    _mm_store_ss(&hit->v[i], _mm_mul_ss(detv, inv_det));
                    hit->intr[i] = 1;
                }
            }
        }
    }
}

void precompute(const float* vertices,
                const unsigned int* indices,
                precomp_tri* precomps,
                unsigned int num_tris)
{

    for (int i = 0; i < num_tris; i++)
    {
        const __m128 a = _mm_load_ps(vertices + 4 * indices[3 * i]);
        const __m128 b = _mm_load_ps(vertices + 4 * indices[3 * i + 1]);
        const __m128 c = _mm_load_ps(vertices + 4 * indices[3 * i + 2]);

        const __m128 ab = _mm_sub_ps(b, a);
        const __m128 ac = _mm_sub_ps(c, a);

        const __m128 n = sse_cross(ab, ac);
        __m128 inv_n2 = sse_recip(sse_length2(n));
        inv_n2 = _mm_shuffle_ps(inv_n2, inv_n2, _MM_SHUFFLE(0, 0, 0, 0));

        __m128 u = sse_cross(ac, n);
        __m128 v = sse_cross(n, ab);
        u = _mm_mul_ps(u, inv_n2);
        v = _mm_mul_ps(v, inv_n2);

        const __m128 dotn = sse_dot3(n, a);
        const __m128 dotu = sse_negate(sse_dot3(u, a));
        const __m128 dotv = sse_negate(sse_dot3(v, a));

        _mm_store_ps(precomps[i].n, n);
        _mm_store_ps(precomps[i].u, u);
        _mm_store_ps(precomps[i].v, v);

        _mm_store_ss(precomps[i].n + 3, dotn);
        _mm_store_ss(precomps[i].u + 3, dotu);
        _mm_store_ss(precomps[i].v + 3, dotv);
    }
}
