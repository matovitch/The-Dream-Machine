#include "shader.h"
#include "linalg.h"
#include "sse.h"
#include "mem.h"
#include <math.h>
#include <xmmintrin.h>
#include <emmintrin.h>

#define NUM_CELLS 8

__m128 interpolate_normals(__m128 u, __m128 v, __m128 t,
                           const face* triangle,
                           const shader_info* shader)
{
    const __m128 n0 = _mm_mul_ps(_mm_load_ps(shader->normals + 4 * triangle->normals[0]), u);
    const __m128 n1 = _mm_mul_ps(_mm_load_ps(shader->normals + 4 * triangle->normals[1]), v);
    const __m128 n2 = _mm_mul_ps(_mm_load_ps(shader->normals + 4 * triangle->normals[2]), t);

    return sse_normalize(_mm_add_ps(n0, _mm_add_ps(n1, n2)));
}

__m128 interpolate_texcoords(__m128 u, __m128 v, __m128 t,
                             const face* triangle,
                             const shader_info* shader)
{
    const __m128 tex0 = _mm_mul_ps(_mm_loadu_ps(shader->texcoords + 2 * triangle->texcoords[0]), u);
    const __m128 tex1 = _mm_mul_ps(_mm_loadu_ps(shader->texcoords + 2 * triangle->texcoords[1]), v);
    const __m128 tex2 = _mm_mul_ps(_mm_loadu_ps(shader->texcoords + 2 * triangle->texcoords[2]), t);

    return _mm_add_ps(tex0, _mm_add_ps(tex1, tex2));
}

__m128 texture_value(const rgb* texture, unsigned int x, unsigned int y, unsigned int w)
{
    const rgb* color = texture + y * w + x;
    return _mm_div_ps(_mm_cvtepi32_ps(_mm_set_epi32(color->a, color->b, color->g, color->r)),
                      _mm_set1_ps(255.0f));
} 

__m128 bilinear_filtering(const float* texcoords,
                          const shader_info* shader,
                          const material* mat)
{
    float tex_x, tex_y;
    if (mat->tex_mode & TEX_REPEAT) {
        float tmp;
        tex_x = modff(texcoords[0], &tmp);
        tex_y = modff(texcoords[1], &tmp);
        if (tex_x < 0) tex_x = 1 + tex_x;
        if (tex_y < 0) tex_y = 1 + tex_y;
    } else {
        tex_x = (texcoords[0] < 0) ? 0 : ((texcoords[0] > 1) ? 1 : texcoords[0]);
        tex_y = (texcoords[1] < 0) ? 0 : ((texcoords[1] > 1) ? 1 : texcoords[1]);
    }

    tex_x *= mat->tex_w - 0.5;
    tex_y *= mat->tex_h - 0.5;

    const unsigned int xmin = floor(tex_x);
    const unsigned int ymin = floor(tex_y);

    const __m128 near_col0 = texture_value(mat->texture, xmin, ymin, mat->tex_w);
    const __m128 near_col1 = texture_value(mat->texture, xmin + 1, ymin, mat->tex_w);
    const __m128 near_col2 = texture_value(mat->texture, xmin, ymin + 1, mat->tex_w);
    const __m128 near_col3 = texture_value(mat->texture, xmin + 1, ymin + 1, mat->tex_w);

    const __m128 u = _mm_set1_ps(tex_x - xmin);
    const __m128 v = _mm_set1_ps(tex_y - ymin);
    const __m128 u_opp = _mm_sub_ps(_mm_set1_ps(1.0f), u);
    const __m128 v_opp = _mm_sub_ps(_mm_set1_ps(1.0f), v);

    const __m128 result_top = _mm_add_ps(_mm_mul_ps(near_col0, u_opp), _mm_mul_ps(near_col1, u));
    const __m128 result_bottom = _mm_add_ps(_mm_mul_ps(near_col2, u_opp), _mm_mul_ps(near_col3, u));

    return _mm_add_ps(_mm_mul_ps(result_top, v_opp), _mm_mul_ps(result_bottom, v));
}

void phong_shader(const packet_hit* hits,
                  const ray_packet* packets,
                  const packet_hit* shadow_hits,
                  const ray_packet* shadow_packets,
                  unsigned int num_packets,
                  const shader_info* shader,
                  rgb* result)
{
    for (int i = 0; i < num_packets; i++) {
        for (int j = 0; j < 4; j++) {
            /* Skip the ray if it doesn't hit anything */
            if (hits[i].intr[j] < 0) {
                result[i * 4 + j].r = shader->bg_color.r;
                result[i * 4 + j].g = shader->bg_color.g;
                result[i * 4 + j].b = shader->bg_color.b;
                result[i * 4 + j].a = shader->bg_color.a;
                continue;
            }

            const face* triangle = shader->faces + hits[i].intr[j];
            const material* mat = shader->materials + triangle->material;

            /* Barycentric coordinates */
            const __m128 v = _mm_load1_ps(&hits[i].u[j]);
            const __m128 t = _mm_load1_ps(&hits[i].v[j]);
            const __m128 u = _mm_sub_ps(_mm_set1_ps(1), _mm_add_ps(v, t));

            /* Interpolate the normal */
            const __m128 avg_normal = interpolate_normals(u, v, t, triangle, shader);

            /* Get the texture color */
            __m128 tex_color = _mm_set1_ps(1.0f);
            if (mat->texture) {
                /* Interpolate the texture coordinates */
                float SSE_ALIGN(texcoords[4]);
                __m128 sse_texture = interpolate_texcoords(u, v, t, triangle, shader);
                _mm_store_ps(texcoords, sse_texture);
                tex_color = bilinear_filtering(texcoords, shader, mat);
            }

            __m128 partial = _mm_mul_ps(_mm_loadu_ps(shader->ambient), _mm_loadu_ps(mat->ambient));

            /* Initialise view vector */
            const __m128 dir_to_eye = sse_negate(sse_normalize(_mm_load_ps(packets[i].dir + 4 * j)));

            /* Sum each light contribution */
            for (int l = 0; l < shader->num_lights; l++) {
                if (shadow_hits[l * num_packets + i].intr[j] >= 0) continue;

                const float* lm = shadow_packets[l * num_packets + i].dir + 4 * j;
                const __m128 dir_to_light = sse_negate(sse_normalize(_mm_load_ps(lm)));

                /* Diffuse component */
                __m128 dot_d = _mm_max_ss(sse_dot3(dir_to_light, avg_normal), _mm_setzero_ps());
                dot_d = _mm_shuffle_ps(dot_d, dot_d, _MM_SHUFFLE(0, 0, 0, 0));
                const __m128 diff = _mm_mul_ps(dot_d, _mm_mul_ps(_mm_loadu_ps(mat->diffuse), tex_color));
                partial = _mm_add_ps(partial, _mm_mul_ps(diff, _mm_loadu_ps(shader->lights[l].diffuse)));

                /* Specular component */
                __m128 rm = _mm_mul_ps(dot_d, avg_normal);
                rm = _mm_sub_ps(_mm_add_ps(rm, rm), dir_to_light);
                const __m128 dot_s = _mm_max_ss(sse_dot3(rm, dir_to_eye), _mm_setzero_ps());
                 float pdot_s;
                _mm_store_ss(&pdot_s, dot_s);
                pdot_s = powf(pdot_s, mat->alpha);

                const __m128 spec = _mm_mul_ps(_mm_loadu_ps(mat->specular), _mm_load1_ps(&pdot_s));
                partial = _mm_add_ps(partial, _mm_mul_ps(spec, _mm_loadu_ps(shader->lights[l].specular)));
            }

            const __m128 sat = _mm_set1_ps(255.0f);
            partial = _mm_mul_ps(partial, sat);
            union {
                __m128i vector;
                unsigned int array[4];
            } pixel;
            
            pixel.vector = _mm_cvttps_epi32(_mm_min_ps(partial, sat));

            result[i * 4 + j].r = pixel.array[0];
            result[i * 4 + j].g = pixel.array[1];
            result[i * 4 + j].b = pixel.array[2];
            result[i * 4 + j].a = pixel.array[3];
        }
    }
}

void cel_shader(const packet_hit* hits,
                const ray_packet* packets,
                const packet_hit* shadow_hits,
                const ray_packet* shadow_packets,
                unsigned int num_packets,
                const shader_info* shader,
                rgb* result)
{
    static const float values[NUM_CELLS] =
    {
        0.0f, 0.1f, 0.3f, 0.4f,
        0.6f, 0.8f, 1.0f, 1.0f
    };

    const __m128 num_cells = _mm_set_ss((float)(NUM_CELLS - 1));

    for (int i = 0; i < num_packets; i++) {
        for (int j = 0; j < 4; j++) {
            /* Skip the ray if it doesn't hit anything */
            if (hits[i].intr[j] < 0) {
                result[i * 4 + j].r = shader->bg_color.r;
                result[i * 4 + j].g = shader->bg_color.g;
                result[i * 4 + j].b = shader->bg_color.b;
                result[i * 4 + j].a = shader->bg_color.a;
                continue;
            }

            const face* triangle = shader->faces + hits[i].intr[j];
            const material* mat = shader->materials + triangle->material;

            /* Barycentric coordinates */
            const __m128 v = _mm_load1_ps(&hits[i].u[j]);
            const __m128 t = _mm_load1_ps(&hits[i].v[j]);
            const __m128 u = _mm_sub_ps(_mm_set1_ps(1), _mm_add_ps(v, t));

            /* Interpolate the normal */
            const __m128 avg_normal = interpolate_normals(u, v, t, triangle, shader);

            /* Get the texture color */
            __m128 tex_color = _mm_set1_ps(1.0f);
            if (mat->texture) {
                /* Interpolate the texture coordinates */
                float SSE_ALIGN(texcoords[4]);
                __m128 sse_texture = interpolate_texcoords(u, v, t, triangle, shader);
                _mm_store_ps(texcoords, sse_texture);
                tex_color = bilinear_filtering(texcoords, shader, mat);
            }

            __m128 partial = _mm_mul_ps(_mm_loadu_ps(shader->ambient), _mm_loadu_ps(mat->ambient));

            /* Sum each light contribution */
            for (int l = 0; l < shader->num_lights; l++) {
                if (shadow_hits[l * num_packets + i].intr[j] >= 0) continue;

                const float* lm = shadow_packets[l * num_packets + i].dir + 4 * j;
                const __m128 dir_to_light = sse_negate(sse_normalize(_mm_load_ps(lm)));

                /* Diffuse component */
                __m128 dot_d = _mm_max_ss(sse_dot3(dir_to_light, avg_normal), _mm_setzero_ps());
                dot_d = _mm_load_ss(values + _mm_cvtss_si32(_mm_mul_ss(dot_d, num_cells)));
                dot_d = _mm_shuffle_ps(dot_d, dot_d, _MM_SHUFFLE(0, 0, 0, 0));
                
                const __m128 diff = _mm_mul_ps(dot_d, _mm_mul_ps(_mm_loadu_ps(mat->diffuse), tex_color));
                partial = _mm_add_ps(partial, _mm_mul_ps(diff, _mm_loadu_ps(shader->lights[l].diffuse)));
            }

            const __m128 sat = _mm_set1_ps(256.0f);
            partial = _mm_mul_ps(partial, sat);
            union {
                __m128i vector;
                unsigned int array[4];
            } pixel;
            
            pixel.vector = _mm_cvttps_epi32(_mm_min_ps(partial, sat));

            result[i * 4 + j].r = pixel.array[0];
            result[i * 4 + j].g = pixel.array[1];
            result[i * 4 + j].b = pixel.array[2];
            result[i * 4 + j].a = pixel.array[3];
        }
    }
}

/*__m128i ssei_negate(__m128i a)
{
    return _mm_xor_si128(_mm_set1_epi16(0x8000), a);
}

__m128i intensity(__m128i set)
{

    __m128i shuf1 = _mm_add_epi32(_mm_shufflehi_epi16(set, _MM_SHUFFLE(0, 3, 2, 1)),
                                  _mm_shufflelo_epi16(set, _MM_SHUFFLE(0, 3, 2, 1)));

    __m128i shuf2 = _mm_add_epi32(_mm_shufflehi_epi16(set, _MM_SHUFFLE(1, 0, 3, 2)),
                                  _mm_shufflelo_epi16(set, _MM_SHUFFLE(1, 0, 3, 2)));

    __m128i shuf3 = _mm_add_epi32(_mm_shufflehi_epi16(set, _MM_SHUFFLE(2, 1, 0, 3)),
                                  _mm_shufflelo_epi16(set, _MM_SHUFFLE(2, 1, 0, 3)));

    return _mm_add_epi32(_mm_add_epi32(set, shuf1), _mm_add_epi32(shuf2, shuf3));
}


void sobel_filter(rgb* img, unsigned int width, unsigned int height)
{
    for (int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            __m128i im1p1_jm1m1 m_set_epi16(rgb[(i - 1) * width + (j - 1)].r,
                                            rgb[(i - 1) * width + (j - 1)].g,
                                            rgb[(i - 1) * width + (j - 1)].b,
                                            rgb[(i - 1) * width + (j - 1)].a,
                                            rgb[(i + 1) * width + (j + 1)].r,
                                            rgb[(i + 1) * width + (j + 1)].g,
                                            rgb[(i + 1) * width + (j + 1)].b,
                                            rgb[(i + 1) * width + (j + 1)].a);

            __m128i im1p1_jp1p1 m_set_epi16(rgb[(i - 1) * width + (j + 1)].r,
                                            rgb[(i - 1) * width + (j + 1)].g,
                                            rgb[(i - 1) * width + (j + 1)].b,
                                            rgb[(i - 1) * width + (j + 1)].a,
                                            rgb[(i + 1) * width + (j + 1)].r,
                                            rgb[(i + 1) * width + (j + 1)].g,
                                            rgb[(i + 1) * width + (j + 1)].b,
                                            rgb[(i + 1) * width + (j + 1)].a);


             __m128i iz0m1_jm1z0 m_set_epi16(rgb[i * width + (j + 1)].r,
                                             rgb[i * width + (j + 1)].g,
                                             rgb[i * width + (j + 1)].b,
                                             rgb[i * width + (j + 1)].a,
                                             rgb[(i - 1) * width + j].r,
                                             rgb[(i - 1) * width + j].g,
                                             rgb[(i - 1) * width + j].b,
                                             rgb[(i - 1) * width + j].a);

            __m128i ip1z0_jz0p1 m_set_epi16(rgb[(i + 1) * width + j].r,
                                        rgb[(i + 1) * width + j].g,
                                        rgb[(i + 1) * width + j].b,
                                        rgb[(i + 1) * width + j].a,
                                        rgb[i * width + (j + 1)].r,
                                        rgb[i * width + (j + 1)].g,
                                        rgb[i * width + (j + 1)].b,
                                        rgb[i * width + (j + 1)].a);

            im1_jm1 = intensity(im1_jm1);
            im1_jz0 = intensity(im1_jz0);
            im1_jp1 = intensity(im1_jp1);
            iz0_jm1 = intensity(iz0_jm1);
            iz0_jp1 = intensity(iz0_jp1);
            ip1_jm1 = intensity(ip1_jm1);
            ip1_jz0 = intensity(ip1_jz0);
            ip1_jp1 = intensity(ip1_jp1);
                

            const __m128i gx_m1 = _mm_add_epi32(im1_jm1, ssei_negate(im1_jp1));
            const __m128i gx_z0 = _mm_add_epi32(iz0_jm1, ssei_negate(iz0_jp1));
            const __m128i gx_p1 = _mm_add_epi32(ip1_jm1, ssei_negate(ip1_jp1));

            const __m128i gy_m1 = _mm_add_epi32(im1_jm1, ssei_negate(ip1_jm1));
            const __m128i gy_z0 = _mm_add_epi32(iz0_jm1, ssei_negate(iz0_jp1));
            const __m128i gx_p1 = _mm_add_epi32(im1_jp1, ssei_negate(ip1_jp1));

            const __m128i gx = _mm_add_epi32(_mm_add_epi32(gx_m1, gx_z0), 
                                             _mm_add_epi32(gx_p1, gx_z0));

            const __m128i gy = _mm_add_epi32(_mm_add_epi32(gy_m1, gy_z0), 
                                             _mm_add_epi32(gy_p1, gy_z0));

            //const __m128 grad_length1 = _mm_add_ps(gx, ty); //error number of bits

            int value = gx + gy;
        }
    }
}*/

