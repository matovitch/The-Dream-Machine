#include "intr.h"
#include "mem.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <float.h>

#define COUNT 1000000

int main(int argc, char** argv)
{
    srand((int)clock() + (int)time(NULL));

    ray_packet* packets = aligned_malloc(sizeof(ray_packet) * COUNT, 16);
    for (int i = 0; i < COUNT; i++) {
        for (int j = 0; j < 4; j++) {
            packets[i].dir[j] = (float)rand() / (float)RAND_MAX;
            packets[i].org[j] = 3 * (float)rand() / (float)RAND_MAX - 1;
            packets[i].inv_dir[j] = 1.0f / packets[i].dir[j];
        }
    }

    int hits[4];
    unsigned long num = 0;
    const float SSE_ALIGN(box_min[4]) = {0, 0, 0, 0};
    const float SSE_ALIGN(box_max[4]) = {1, 1, 1, 1};
    const float SSE_ALIGN(tmin[4]) = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};

    clock_t ck = clock();
    for (int i = 0; i < COUNT; i++) {
        intr_packet_box(packets + i, box_min, box_max, tmin, hits);

        num = (hits[0] != 0) ? num + 1 : num;
        num = (hits[1] != 0) ? num + 1 : num;
        num = (hits[2] != 0) ? num + 1 : num;
        num = (hits[3] != 0) ? num + 1 : num;
    }

    aligned_free(packets);

    printf("%lu hits\n", num);
    printf("%ld ms\n", (clock() - ck) * 1000 / CLOCKS_PER_SEC);

    return 0;
}
