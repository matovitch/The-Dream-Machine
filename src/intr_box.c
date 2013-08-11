#include "intr.h"
#include "linalg.h"
#include <stdio.h>

void intr_packet_box(const ray_packet* packet,
                     const float* box_min,
                     const float* box_max,
                     const float* prev_tmin,
                     int* hit)
{
    for (int i = 0; i < 4; i++) {
        const float* org = packet->org + i * 4;
        const float* inv = packet->inv_dir + i * 4;
        float tmin[4], tmax[4];

        vector3_subtract(box_max, org, tmax);
        vector3_subtract(box_min, org, tmin);
        vector3_scale3(tmax, inv, tmax);
        vector3_scale3(tmin, inv, tmin);

        float vmin[4], vmax[4];
        vector3_max(tmax, tmin, vmax);
        vector3_min(tmax, tmin, vmin);

        vmax[0] = (vmax[0] < vmax[1]) ? vmax[0] : vmax[1];
        vmax[0] = (vmax[0] < vmax[2]) ? vmax[0] : vmax[2];

        vmin[0] = (vmin[0] > vmin[1]) ? vmin[0] : vmin[1];
        vmin[0] = (vmin[0] > vmin[2]) ? vmin[0] : vmin[2];

        hit[i] = (vmin[0] <= vmax[0]) && (vmax[0] >= 0) && (vmin[0] <= prev_tmin[i]);
    }
}
