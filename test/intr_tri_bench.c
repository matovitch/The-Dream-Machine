#include "intr.h"
#include "mem.h"
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <stdio.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define NUM_PACKETS 1000000

float dir_inv(float dir)
{
    if (dir == 0)
        return FLT_MAX;
    else
        return 1. / dir;
}

int main(int argc, char** argv)
{
    const float SSE_ALIGN(vertices[16]) = {1, 0, 0, 1,
                                           0, 1, 0, 1,
                                           0, 0, 1, 1};
    const unsigned int indices[3] = {0, 1, 2};
    const unsigned int num_tris = 1;
    precomp_tri SSE_ALIGN(precomps[num_tris]);

    precompute(vertices, indices, precomps, num_tris);

    ray_packet* packets = aligned_malloc(NUM_PACKETS * sizeof(ray_packet), 16);

    srand(time(NULL));
    for (int i = 0; i < NUM_PACKETS; i++) {
        for (int j = 0; j < 4; j++) {
            float theta = (float)rand() / (float)RAND_MAX * 2. * M_PI;
            float phi = (float)rand() / (float)RAND_MAX * M_PI;

            packets[i].org[4 * j + 0] = 3 * cos(theta) * sin(phi);
            packets[i].org[4 * j + 1] = 3 * sin(theta) * sin(phi);
            packets[i].org[4 * j + 2] = 3 * cos(phi);
            packets[i].org[4 * j + 3] = 1;

            packets[i].dir[4 * j + 0] = -packets[i].org[4 * j + 0];
            packets[i].dir[4 * j + 1] = -packets[i].org[4 * j + 1];
            packets[i].dir[4 * j + 2] = -packets[i].org[4 * j + 2];
            packets[i].dir[4 * j + 3] = 1;

            packets[i].inv_dir[4 * j + 0] = dir_inv(packets[i].dir[4 * j + 0]);
            packets[i].inv_dir[4 * j + 1] = dir_inv(packets[i].dir[4 * j + 1]);
            packets[i].inv_dir[4 * j + 2] = dir_inv(packets[i].dir[4 * j + 2]);
            packets[i].inv_dir[4 * j + 3] = 1;
        }
    }

    packet_hit SSE_ALIGN(hit);

    clock_t ck = clock();

    for (int i = 0; i < NUM_PACKETS; i++)
        intr_packet_tri(packets + i, precomps, &hit);

    aligned_free(packets);

    printf("%d rays\n", 4 * NUM_PACKETS);
    printf("%ld ms\n", (clock() - ck) * 1000 / CLOCKS_PER_SEC);

    return 0;
}
