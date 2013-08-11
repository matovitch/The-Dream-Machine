#include "intr.h"
#include "bvh.h"
#include "mem.h"
#include <float.h>
#include <stdio.h>

#define NUM_TRI 3

int main(int argc, char** argv)
{
    float SSE_ALIGN(vertices[12 * NUM_TRI]);
    unsigned int indices[3 * NUM_TRI];

    /* Stack of triangles */
    for (int i = 0; i < NUM_TRI; i++) {
        vertices[12 * i + 0] = 0;
        vertices[12 * i + 1] = 0;
        vertices[12 * i + 2] = i;
        vertices[12 * i + 3] = 1;

        vertices[12 * i + 4] = 1;
        vertices[12 * i + 5] = 0;
        vertices[12 * i + 6] = i;
        vertices[12 * i + 7] = 1;

        vertices[12 * i + 8] = 0;
        vertices[12 * i + 9] = 1;
        vertices[12 * i + 10] = i;
        vertices[12 * i + 11] = 1;
    }

    for (int i = 0; i < 3 * NUM_TRI; i++)
        indices[i] = i;

    bvh_obj bvh;
    build_bvh(indices, NUM_TRI, vertices, &bvh);

    /* Stupid frustum that culls nothing */
    float frustum[4 * 6] = {1, 0, 0, 9,
                            1, 0, 0, 9,
                            1, 0, 0, 9,
                            1, 0, 0, 9,
                            1, 0, 0, 9,
                            1, 0, 0, 9};

    ray_packet SSE_ALIGN(packet) = 
    { 
        /* origins */                           
        {0.1f, 0.1f, 1000, 1,
         0.1f, 0.1f, 1000, 1,
         0.1f, 0.1f, 1000, 1,
         0.1f, 0.1f, 1000, 1},
        /* directions */
        {0, 0, -1, 0,
         0, 0, -1, 0,
         0, 0, -1, 0,
         0, 0, -1, 0},
        /* inverse direction */
        {FLT_MAX, FLT_MAX, -1, FLT_MAX,
        FLT_MAX, FLT_MAX, -1, FLT_MAX,
        FLT_MAX, FLT_MAX, -1, FLT_MAX,
        FLT_MAX, FLT_MAX, -1, FLT_MAX}
    };

    precomp_tri SSE_ALIGN(precomps[NUM_TRI]);
    precompute(vertices, indices, precomps, NUM_TRI);

    packet_hit SSE_ALIGN(hit);
    closest_intr_bvh(&packet, 1, frustum, &bvh, precomps, &hit);

    printf("%d\n", hit.intr[0]);

    return 0;
}