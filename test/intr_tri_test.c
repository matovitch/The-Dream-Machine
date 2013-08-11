#include "intr.h"
#include "mem.h"
#include <stdio.h>
#include <float.h>

int main(int argc, char** argv)
{
    const float SSE_ALIGN(vertices[16]) = {0.5, 0.5, 0, 1,
                                           1.5, 2.5, 1.0, 1,
                                           0.5, 1.5, 0, 1};
    const unsigned int indices[3] = {0, 1, 2};
    const unsigned int num_tris = 1;
    precomp_tri SSE_ALIGN(precomps[num_tris]);

    precompute(vertices, indices, precomps, num_tris);

    printf("n : %f %f %f %f\n", precomps->n[0], precomps->n[1], precomps->n[2], precomps->n[3]);
    printf("u : %f %f %f %f\n", precomps->u[0], precomps->u[1], precomps->u[2], precomps->u[3]);
    printf("v : %f %f %f %f\n", precomps->v[0], precomps->v[1], precomps->v[2], precomps->v[3]);

    ray_packet SSE_ALIGN(packet) = 
	{ 
		/* origins */ 							
		{-30.0, 0.5, -100.0, 1,
         0.2, 0.2, -3, 1,
         0.4, 0.4, -3, 1,
         0.6, 0.6, -3, 1},
		/* directions */
		{0, 0, 1, 0,
         0, 0, 1, 0,
         0, 0, 1, 0,
         0, 0, 1, 0},
		/* inverse direction */
		{FLT_MAX, FLT_MAX, 1, 1,
         FLT_MAX, FLT_MAX, 1, 1,
         FLT_MAX, FLT_MAX, 1, 1,
         FLT_MAX, FLT_MAX, 1, 1}
	};

    packet_hit SSE_ALIGN(hit);

    hit.t[0] = FLT_MAX;
    hit.t[1] = FLT_MAX;
    hit.t[2] = FLT_MAX;
    hit.t[3] = FLT_MAX;

	intr_packet_tri(&packet, precomps, &hit);

    for (int i = 0; i < 4; i++)
        if (hit.intr[i] != 0)
            printf("Le rayon %d touche en : t = %f, u = %f, v = %f\n",
                              i, hit.t[i], hit.u[i], hit.v[i]);

	return 0;
}
