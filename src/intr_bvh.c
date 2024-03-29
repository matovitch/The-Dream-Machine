#include "intr.h"
#include "mem.h"
#include "linalg.h"
#include <float.h>
#include <math.h>

typedef struct
{
    const bvh_node* node;
    unsigned int active_ray;
} stack_node;

unsigned int get_first_hit(const ray_packet* packets,
                           const packet_hit* hits,
                           unsigned int num_packets,
                           const float* frustum,
                           unsigned int active,
                           const bvh_node* node)
{
    /* Then test with the active packet */
    int box_hits[4];
    intr_packet_box(packets + active, node->box_min, node->box_max, hits[active].t, box_hits);
    if (box_hits[0] || box_hits[1] || box_hits[2] || box_hits[3])
        return active;

    /* Test with the frustum first */
    float SSE_ALIGN(box_min[4]);
    float SSE_ALIGN(box_max[4]);
    vector3_copy(node->box_min, box_min);
    box_min[3] = 1.0f;
    vector3_copy(node->box_max, box_max);
    box_max[3] = 1.0f;
    
    if (!intr_frustum_box(frustum, box_min, box_max))
        return num_packets;

    /* Test all packets */
    for (unsigned int i = active + 1; i < num_packets; i++) {
        intr_packet_box(packets + i, node->box_min, node->box_max, hits[i].t, box_hits);
        if (box_hits[0] || box_hits[1] || box_hits[2] || box_hits[3]) {
            return i;
        }
    }

    return num_packets;
}

unsigned int get_last_hit(const ray_packet* packets,
                          const packet_hit* hits,
                          unsigned int num_packets,
                          const float* frustum,
                          unsigned int active,
                          const bvh_node* node)
{
    /* Find last ray hit */
    int box_hits[4];
    for (unsigned int i = num_packets - 1; i > active; i--) {
        intr_packet_box(packets + i, node->box_min, node->box_max, hits[i].t, box_hits);
        if (box_hits[0] || box_hits[1] || box_hits[2] || box_hits[3]) {
            return i + 1;
        }
    }

    return active + 1;
}

void closest_intr_node(const bvh_obj* bvh,
                       const bvh_node* node,
                       const precomp_tri* tris,
                       const ray_packet* packets,
                       unsigned int first,
                       unsigned int last,
                       packet_hit* hits)
{
    /* Loop over all triangles in the node */
    for (unsigned int i = first; i < last; i++) {
        const ray_packet* packet = packets + i;

        for (unsigned int j = 0; j < node->num_tris; j++) {
            const unsigned int tri_id = bvh->tri_ids[node->node_data.tri_id + j];
            packet_hit SSE_ALIGN(hit);

            hit.t[0] = hits[i].t[0];
            hit.t[1] = hits[i].t[1];
            hit.t[2] = hits[i].t[2];
            hit.t[3] = hits[i].t[3];
            
            intr_packet_tri(packet, tris + tri_id, &hit);

            /* Update the ray-triangle intersection result */
            for (int k = 0; k < 4; k++) { 
                if (hit.intr[k] && hit.t[k] < hits[i].t[k]) {
                    hits[i].t[k] = hit.t[k];
                    hits[i].u[k] = hit.u[k];
                    hits[i].v[k] = hit.v[k];
                    hits[i].intr[k] = tri_id;
                }
            }
        }
    }
}

void closest_intr_bvh(const ray_packet* packets,
                      unsigned int num_packets,
                      const float* frustum,
                      const bvh_obj* bvh,
                      const precomp_tri* tris,
                      packet_hit* hits)
{
    stack_node stack[bvh->depth + 1];

    for (unsigned int i = 0; i < num_packets; i++) {
		hits[i].t[0] = FLT_MAX;
		hits[i].t[1] = FLT_MAX;
		hits[i].t[2] = FLT_MAX;
		hits[i].t[3] = FLT_MAX;
		
        hits[i].intr[0] = -1;
        hits[i].intr[1] = -1;
        hits[i].intr[2] = -1;
        hits[i].intr[3] = -1;
    }

    /* Push the first node onto the stack */
    unsigned int cur_depth = 1;
    stack[0].node = bvh->root;
    stack[0].active_ray = 0;

    while (cur_depth > 0) {
        /* Pop the next element */
        cur_depth--;
        const bvh_node* node = stack[cur_depth].node;

        unsigned int first = get_first_hit(packets, hits, num_packets, frustum, stack[cur_depth].active_ray, node);
        if (first < num_packets) {
            if (node->num_tris == 0) {
                /* At least one packet intersected the node, go through children */
                const int order = (packets[first].dir[node->axis] > 0) ? node->order : 1 - node->order; 
                const bvh_node* child0 = bvh->root + node->node_data.child + order;
                const bvh_node* child1 = bvh->root + node->node_data.child + 1 - order;

                stack[cur_depth].active_ray = first;
                stack[cur_depth++].node = child1;

                stack[cur_depth].active_ray = first;
                stack[cur_depth++].node = child0;
            } else {
                unsigned int last = get_last_hit(packets, hits, num_packets, frustum, first, node);
                closest_intr_node(bvh, node, tris, packets, first, last, hits);
            }
        }
    }
}

int dead_packet(const packet_hit* hit)
{
    /* A packet is dead if all the rays in it have touched a triangle */
    return !((hit->intr[0] & 0x80000000) | (hit->intr[1] & 0x80000000) |
             (hit->intr[2] & 0x80000000) | (hit->intr[3] & 0x80000000));
}

void first_intr_node(const bvh_obj* bvh,
                     const bvh_node* node,
                     const precomp_tri* tris,
                     const ray_packet* packets,
                     unsigned int first,
                     unsigned int last,
                     packet_hit* hits,
                     const packet_hit* prev_hits)
{
    /* For each active packet */
    for (unsigned int i = first; i < last; i++) {
        const ray_packet* packet = packets + i;
        
        /* Loop over all triangles in the node */
        for (unsigned int j = 0; j < node->num_tris; j++) {
            /* If the packet has already hit a triangle, skip it */
            if (dead_packet(hits + i)) break;

            const unsigned int tri_id = bvh->tri_ids[node->node_data.tri_id + j];

            packet_hit SSE_ALIGN(hit);

            hit.t[0] = 1.0f;
            hit.t[1] = 1.0f;
            hit.t[2] = 1.0f;
            hit.t[3] = 1.0f;

            intr_packet_tri(packet, tris + tri_id, &hit);

            /* Update the ray-triangle intersection result */
            for (int k = 0; k < 4; k++) { 
                if (tri_id != prev_hits[i].intr[k] &&
                    hit.intr[k] && hit.t[k] < 1) {
                    hits[i].t[k] = hit.t[k];
                    hits[i].u[k] = hit.u[k];
                    hits[i].v[k] = hit.v[k];
                    hits[i].intr[k] = tri_id;
                }
            }
        }
    }
}

void first_intr_bvh(const ray_packet* packets,
                    unsigned int num_packets,
                    const float* frustum,
                    const bvh_obj* bvh,
                    const precomp_tri* tris,
                    packet_hit* hits,
                    const packet_hit* prev_hits)
{
    stack_node stack[bvh->depth + 1];

    for (unsigned int i = 0; i < num_packets; i++) {
        hits[i].t[0] = FLT_MAX;
		hits[i].t[1] = FLT_MAX;
		hits[i].t[2] = FLT_MAX;
		hits[i].t[3] = FLT_MAX;
		
        hits[i].intr[0] = -1;
        hits[i].intr[1] = -1;
        hits[i].intr[2] = -1;
        hits[i].intr[3] = -1;
    }

    /* Push the first node onto the stack */
    unsigned int cur_depth = 1;
    stack[0].node = bvh->root;
    stack[0].active_ray = 0;

    /* Store the first and last alive ray */
    unsigned int first_alive = 0;
    unsigned int last_alive = num_packets;

    while (cur_depth > 0 && last_alive > first_alive) {
        /* Pop the next element */
        cur_depth--;
        const bvh_node* node = stack[cur_depth].node;
        unsigned int first = (first_alive > stack[cur_depth].active_ray)
                             ? first_alive
                             : stack[cur_depth].active_ray;

        /* Get the first hit. Start at max(first_alive, first_active) */
        first = get_first_hit(packets, hits, last_alive, frustum, first, node);

        if (first < last_alive) {
            if (node->num_tris == 0) {
                /* At least one packet intersected the node, go through children */
                const int order = (packets[first].dir[node->axis] > 0) ? node->order : 1 - node->order; 
                const bvh_node* child0 = bvh->root + node->node_data.child + order;
                const bvh_node* child1 = bvh->root + node->node_data.child + 1 - order;

                stack[cur_depth].active_ray = first;
                stack[cur_depth++].node = child1;

                stack[cur_depth].active_ray = first;
                stack[cur_depth++].node = child0;
            } else {
                unsigned int last = get_last_hit(packets, hits, last_alive, frustum, first, node);
                first_intr_node(bvh, node, tris, packets, first, last, hits, prev_hits);

                /* Change the first and last alive rays */
                while (first_alive < last_alive && dead_packet(hits + first_alive)) {
                    first_alive++;
                }

                while (last_alive > first_alive && dead_packet(hits + last_alive - 1)) {
                    last_alive--;
                }
            }
        }
    }
}
