#include "intr.h"
#include "linalg.h"
#include <math.h>

void intr_packet_tri(const ray_packet* packet,
                     const precomp_tri* precomp,
                     packet_hit* hit)
{
    float det[4];

    hit->intr[0] = 0;
    hit->intr[1] = 0;
    hit->intr[2] = 0;
    hit->intr[3] = 0;

    /* det = d.n */
    det[0] = vector3_dot(packet->dir, precomp->n);
    det[1] = vector3_dot(packet->dir + 4, precomp->n);
    det[2] = vector3_dot(packet->dir + 8, precomp->n);
    det[3] = vector3_dot(packet->dir + 12, precomp->n);

    float oldt[4];
    vector4_copy(hit->t, oldt);

    /* t' = d - o.n */
    hit->t[0] = precomp->n[3] - vector3_dot(packet->org, precomp->n);
    hit->t[1] = precomp->n[3] - vector3_dot(packet->org + 4, precomp->n);
    hit->t[2] = precomp->n[3] - vector3_dot(packet->org + 8, precomp->n);
    hit->t[3] = precomp->n[3] - vector3_dot(packet->org + 12, precomp->n);

    for (int i = 0; i < 4; i++) {
          /* p' = det * o + t' * d */
        if (signbit(hit->t[i]) == signbit(det[i] * oldt[i] - hit->t[i])) {
            float temp[4], P2[4];
            vector3_scale(packet->org + 4 * i, det[i], P2);
            vector3_scale(packet->dir + 4 * i, hit->t[i], temp);
            vector3_add(P2, temp, P2);
            /*u' = p'.u + det.d1*/
            hit->u[i] = vector3_dot(P2, precomp->u) + det[i] * precomp->u[3];

            if (signbit(hit->u[i]) == signbit(det[i] - hit->u[i])) {
                /*v' = p'.v + det.d2*/
                hit->v[i] = vector3_dot(P2, precomp->v) + det[i] * precomp->v[3];

                if(signbit(hit->v[i]) == signbit(det[i] - hit->v[i] - hit->u[i])) {
                    float inv_det = 1.0f / det[i];
                    hit->u[i] *= inv_det;
                    hit->v[i] *= inv_det;
                    hit->t[i] *= inv_det;
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
    for (int i = 0; i < num_tris; i++) {
        const float* a = vertices + 4 * indices[3 * i];
        const float* b = vertices + 4 * indices[3 * i + 1];
        const float* c = vertices + 4 * indices[3 * i + 2];
        float ab[4], ac[4];
        float inv_n2;

        /* n = ab x ac && n[3] = a.n */
        vector3_subtract(b, a, ab);
        vector3_subtract(c, a, ac);
        vector3_cross(ab, ac, precomps[i].n);
        precomps[i].n[3] = vector3_dot(a, precomps[i].n);

        /* inv_n2 = 1/|n^2| */
        inv_n2 = 1.0f / vector3_length2(precomps[i].n);

        /* u = ac x n / |n^2| && u[3] = -a.u */
        vector3_cross(ac, precomps[i].n, precomps[i].u);
        vector3_scale(precomps[i].u, inv_n2, precomps[i].u);
        precomps[i].u[3] = -vector3_dot(a, precomps[i].u);

        /* v = ab x n / |n^2| && v[3] = -a.v */
        vector3_cross(precomps[i].n, ab, precomps[i].v);
        vector3_scale(precomps[i].v, inv_n2, precomps[i].v);
        precomps[i].v[3] = -vector3_dot(a, precomps[i].v);
    }
}
