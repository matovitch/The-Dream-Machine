#include "intr.h"
#include "mem.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#define SHIFT 80


int main(int argc, char** agv)
{
    float SSE_ALIGN(box_min[4]) = {0,0,0,1};
    float SSE_ALIGN(box_max[4]) = {1,1,1,1};
    float SSE_ALIGN(tmin[4]) = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
    const float SSE_ALIGN(inf) = -logf(0.0f);
    int hit[4];

    /* TEST 1 : one packet against the unit cube */

    float orgs[4 * 4] = {-1, 0.99, 0.99, 1,    /* hit center x */
                        0.5, -1, 0.5, 1,     /* hit center y */
                        1, 2, 2, 1,     /* hit center z */
                        -1, 2, 2, 1};        /* miss */

    float dirs[4 * 4] = {1, 0, 0, 1,         /* hit center x */
                        0, 1, 0, 1,          /* hit center y */
                        0, 0, 1, 1,          /* hit center z */
                        1, 0, 0, 1};         /* miss */

    float inv_dirs[4 * 4] = {1, inf, inf, 1,
                            inf, 1, inf, 1,
                            -1, inf, inf , 1,
                            1, inf, inf , 1};

    ray_packet SSE_ALIGN(packet);

    memcpy(packet.org, orgs, sizeof(float) * 4 * 4);
    memcpy(packet.dir, dirs, sizeof(float) * 4 * 4);
    memcpy(packet.inv_dir, inv_dirs, sizeof(float) * 4 * 4);

    intr_packet_box(&packet, box_min, box_max, tmin, hit);

    printf("%d %d %d %d\n", hit[0], hit[1], hit[2], hit[3]);

    /* TEST 2 : moving packet against unit cube again */

	ray_packet SSE_ALIGN(packet2) = 
	{ 
		/* origins */ 							
		{-1, 0.5, -1.50, 1,
		-1, 0.5, -1.55, 1,
		-1, 0.5, -1.60, 1,
		-1, 0.5, -1.65, 1},
		/* directions */
		{1, 0, 0, 1,
		1, 0, 0, 1,
		1, 0, 0, 1,
		1, 0, 0, 1},
		/* inverse direction */
		{1, inf, inf, 1,
		1, inf, inf, 1,
		1, inf, inf, 1,
		1, inf, inf, 1}
	};

	for (int i = 0; i < SHIFT; i++) {
		packet2.org[2] += .05;
		packet2.org[4 + 2] += .05;
		packet2.org[8 + 2] += .05;
		packet2.org[12 + 2] += .05;
	
		intr_packet_box(&packet2, box_min, box_max, tmin, hit);		

		printf("%d %d %d %d    %f\n", hit[0], hit[1], hit[2], hit[3], packet2.org[2]);
	}    
    
    return 0;
}
