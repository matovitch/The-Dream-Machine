/** (C) 2013-2014 MadMann's Company
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

/**
 * \file intr.h
 * \brief Ray intersections functions
 * \author Arsène Pérard-Gayot, Camille Brugel
 * \date 10 juin 2013
 */

#ifndef DREAM_INTR_H
#define DREAM_INTR_H

#include "bvh.h" 

/**
* \struct ray_packet
* 4 rays packet
* 192 bytes aligned */
typedef struct
{
    float org[4 * 4];
    float dir[4 * 4];
    float inv_dir[4 * 4];      /**< Precomputed inverse coordinates of dir */
} ray_packet;

/**
* \struct precomp_tri
* Precomputed triangle
* 48 bytes aligned*/
typedef struct 
{
    float n[4];                 /**< Triangle normal */ 
    float u[4];                 /**< Othogonal plane along an edge */ 
    float v[4];                 /**< Othogonal plane along an edge */
} precomp_tri;

/**
* \struct packet_hit
* Hit result
* 64 bytes aligned*/
typedef struct 
{
    float t[4];               /**< Intersection times */ 
    float u[4];               /**< Barycentric coordinates */ 
    float v[4];               /**< Barycentric coordinates */ 
    int intr[4];              /**< Intersection predicates */ 
} packet_hit;

/** Intersects a triangle with a ray packet
 * \param[in]  packet    Packet of rays (last coordinates of dir = 0.0f, org = 1.0f)
 * \param[in]  precomp   Precomputed triangle
 * \param[out] hit       Result of the intersection
 */
void intr_packet_tri(const ray_packet* packet,
                     const precomp_tri* precomp,
                     packet_hit* hit);

/** Intersects a triangle with a packet
 * \param[in]  vertices   Pointer to the vertices
 * \param[in]  indices    Pointer to the triangles indices
 * \param[out] precomp    Precomputed triangles
 * \param[out] num_tris   Number of triangles
 */
void precompute(const float* vertices,
                const unsigned int* indices,
                precomp_tri* precomps,
                unsigned int num_tris);

/** Intersects a box with a packet
 * \param[in]  ray_packet  Packet to intersect (16 bytes aligned)
 * \param[in]  box_min     Minimal point of the box (16 bytes aligned)
 * \param[in]  box_max     Maximal point of the box (16 bytes aligned)
 * \param[in]  prev_tmin   Previous intersection time (for culling purposes, 16 bytes aligned)
 * \param[out] hit         Contains the result of the intersection
 */
void intr_packet_box(const ray_packet* packet,
                     const float* box_min,
                     const float* box_max,
                     const float* prev_tmin,
                     int* hit);

/** Intersects a frustum with a box
 * \param[in] frustum     Frustum planes (16 bytes aligned, normals pointing outside)
 * \param[in] box_min     Minimal point of the box (16 bytes aligned, last coordinate = 1.0f)
 * \param[in] box_max     Maximal point of the box (16 bytes aligned, last coordinate = 1.0f)
 * \return 0 iff no intersection was found
 */
int intr_frustum_box(const float* frustum,
                     const float* box_min,
                     const float* box_max);


/** Intersects a bvh with several packets, finds the closest intersection
 * \param[in]  packets       List of ray packets (16 bytes aligned)
 * \param[in]  num_packets   Number of packets
 * \param[in]  frustum       Bounding frustum of the packet (16 bytes aligned)
 * \param[in]  bvh           BVH object
 * \param[out] hits          Array of hits
 */
void closest_intr_bvh(const ray_packet* packets,
                      unsigned int num_packets,
                      const float* frustum,
                      const bvh_obj* bvh,
                      const precomp_tri* tris,
                      packet_hit* hits);

/** Intersects a bvh with several packets, stops at the first intersection between (0..1)
 * \param[in]  packets       List of ray packets (16 bytes aligned)
 * \param[in]  num_packets   Number of packets
 * \param[in]  frustum       Bounding frustum of the packet (16 bytes aligned)
 * \param[in]  bvh           BVH object
 * \param[out] hits          Array of hits
 * \param[in]  prev_hits     Previous hits (primary rays hits)
 */
void first_intr_bvh(const ray_packet* packets,
                    unsigned int num_packets,
                    const float* frustum,
                    const bvh_obj* bvh,
                    const precomp_tri* tris,
                    packet_hit* hits,
                    const packet_hit* prev_hits);

#endif // DREAM_INTR_H
