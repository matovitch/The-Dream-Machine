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
 * \file shader.h
 * \brief Shading
 * \author Arsène Pérard-Gayot, Camille Brugel
 * \date 10 juin 2013
 */

#ifndef DREAM_SHADER_H
#define DREAM_SHADER_H

#include "intr.h"

/* Enumerate the differet shading modes */
enum 
{
    TEX_REPEAT = 0x01,
    TEX_CLAMP = 0x02,
    TEX_NEAREST = 0x04,
    TEX_INTERP = 0x08,
};

/* Basic structure for rgb color */
typedef struct
{
    unsigned char r, g, b, a;
} rgb;

/* Material information structure */
typedef struct
{
    float diffuse[4];
    float specular[4];
    float ambient[4];
    float alpha;
    unsigned tex_w, tex_h;
    int tex_mode;
    rgb* texture;
} material;

typedef struct 
{
    float pos[4];
    float diffuse[4];
    float specular[4];
} light;

typedef struct
{
    unsigned int vertices[3];
    unsigned int normals[3];
    unsigned int texcoords[3];
    unsigned int material;
} face;

/* /!\ Normals must be unit length */
typedef struct
{
    float* vertices;
    float* normals;
    float* texcoords;
    material* materials;
    face* faces;
    light* lights;
    unsigned int num_lights;
    float ambient[4];
    rgb bg_color;
} shader_info;

/** Apply Phong shading over an array of ray packets
 * \param[in]  hits             Array of primary ray hits
 * \param[in]  packets          Array of primary ray packets
 * \param[in]  shadow_hits      Array of shadow ray hits
 * \param[in]  shadow_packets   Array of shadow ray packets
 * \param[in]  num_packets      Number of packets
 * \param[in]  shader           Shading info about the mesh
 * \param[out] result           Resulting color array
 */
void phong_shader(const packet_hit* hits,
                  const ray_packet* packet,
                  const packet_hit* shadow_hits,
                  const ray_packet* shadow_packet,
                  unsigned int num_packets,
                  const shader_info* shader,
                  rgb* result);

/** Apply Cel shading over an array of ray packets
 * \param[in]  hits             Array of primary ray hits
 * \param[in]  packets          Array of primary ray packets
 * \param[in]  shadow_hits      Array of shadow ray hits
 * \param[in]  shadow_packets   Array of shadow ray packets
 * \param[in]  num_packets      Number of packets
 * \param[in]  shader           Shading info about the mesh
 * \param[out] result           Resulting color array
 */
void cel_shader(const packet_hit* hits,
                const ray_packet* packets,
                const packet_hit* shadow_hits,
                const ray_packet* shadow_packets,
                unsigned int num_packets,
                const shader_info* shader,
                rgb* result);

void sobel_filter(rgb* img, unsigned int width, unsigned int height);

#endif // DREAM_SHADER_H
