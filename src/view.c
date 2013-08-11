#include "view.h"
#include "linalg.h"
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define COMPUTE_PLANE(a, b, p, r) \
    vector3_cross(b, a, r); \
    (r)[3] = -vector3_dot(r, p);

void rotate_view(float angle,
                 const float* axis,
                 float* pos)
{
    quat temp, quat_view;

    temp[0] = axis[0] * sinf(angle / 2);
    temp[1] = axis[1] * sinf(angle / 2);
    temp[2] = axis[2] * sinf(angle / 2);
    temp[3] = cosf(angle / 2);

    vector3_copy(pos, quat_view);
    quat_view[3] = 0;

    float result[4];
    quat_mult(temp, quat_view, result);
    quat_conj(temp, temp);
    quat_mult(result, temp, pos);
}

void update_view(float tx, float ty,
                 float rx, float ry,
                 float* up,
                 float* eye,
                 float* pos)
{
    float dir[4], right[4];
    vector3_subtract(pos, eye, dir);
    vector3_scale(dir, 1.0f / vector3_length(dir), dir);
    vector3_cross(dir, up, right);

    /* Rotate the view */
    if (rx != 0 || ry != 0) {
        rotate_view(ry, right, dir);
        rotate_view(rx, up, dir);
        pos[0] = eye[0] + dir[0];
        pos[1] = eye[1] + dir[1];
        pos[2] = eye[2] + dir[2];
        pos[3] = 1.0f;

        vector3_cross(right, dir, up);
        vector3_scale(up, 1.0f / vector3_length(up), up);
    }

    /* Translate the eye orthogonally to its dir */
    if (tx != 0 || ty != 0) {
        /* First, translate along x */
        if (tx != 0) {
            vector3_scale(right, -tx, right);
            vector3_add(eye, right, eye);
            vector3_add(pos, right, pos);
        }

        /* Then y */
        if (ty != 0) {
            vector3_scale(dir, ty, dir);
            vector3_add(eye, dir, eye);
            vector3_add(pos, dir, pos);
        }
    }
}

void setup_view_persp(view_info* view,
                      const float* eye,
                      const float* pos,
                      const float* up,
                      float fov, float ratio,
                      float near, float far)
{
    vector3_subtract(pos, eye, view->dir);
    vector3_scale(view->dir, 1.0f / vector3_length(view->dir), view->dir);

    vector3_cross(view->dir, up, view->right);
    vector3_scale(view->right, 1.0f / vector3_length(view->right), view->right);

    vector3_cross(view->right, view->dir, view->up);

    float offset = vector3_dot(view->dir, eye);

    /* Near plane */
    vector3_negate(view->dir, view->near);
    view->near[3] = offset + near;

    /* Far plane */
    vector3_copy(view->dir, view->far);
    view->far[3] = -offset - far;

    view->wnear = near * tanf(fov * M_PI / 180.0f * 0.5f);
    view->hnear = view->wnear / ratio;

    vector3_copy(eye, view->eye);
    view->dnear = near;
}

void build_packets_persp(const view_info* view,
                         ray_packet* packets,
                         unsigned int i, unsigned int j,
                         unsigned int packet_w, unsigned int packet_h,
                         unsigned int view_w, unsigned int view_h)
{
    float point_y[4], right_point[4], up_point[4];
    vector3_scale(view->dir, view->dnear, point_y);
    vector3_add(view->eye, point_y, point_y);

    vector3_scale(view->up, view->hnear * (2 * (float)j / (float)view_h - 1), up_point);
    vector3_scale(view->right, view->wnear * (2 * (float)i / (float)view_w - 1), right_point);
    vector3_add(point_y, up_point, point_y);
    vector3_add(point_y, right_point, point_y);

    float right_offset[4], up_offset[4];
    vector3_scale(view->up, (view->hnear * 2) / (float)view_h, up_offset);
    vector3_scale(view->right, (view->wnear * 2) / (float)view_w, right_offset);

    /* Build packets */
    for (unsigned int y = 0; y < packet_h; y++) {
        float point_x[4];
        vector3_copy(point_y, point_x);

        for (unsigned int x = 0; x < packet_w / 4; x++) {
            ray_packet* packet = packets + y * packet_w / 4 + x;
            for (int k = 0; k < 4; k++) {
                packet->org[k * 4 + 0] = point_x[0];
                packet->org[k * 4 + 1] = point_x[1];
                packet->org[k * 4 + 2] = point_x[2];
                packet->org[k * 4 + 3] = 1.0f;

                vector3_subtract(point_x, view->eye, packet->dir + k * 4);
                packet->dir[k * 4 + 3] = 0.0f;

                packet->inv_dir[k * 4 + 0] = 1.0f / packet->dir[k * 4 + 0];
                packet->inv_dir[k * 4 + 1] = 1.0f / packet->dir[k * 4 + 1];
                packet->inv_dir[k * 4 + 2] = 1.0f / packet->dir[k * 4 + 2];
                packet->inv_dir[k * 4 + 3] = FLT_MAX;
                
                vector3_add(point_x, right_offset, point_x);
            }
        }
        vector3_add(point_y, up_offset, point_y);
    }
}

void build_frustum_persp(const ray_packet* packets,
                         unsigned int packet_w, unsigned int packet_h,
                         const float* near_plane, const float* far_plane,
                         float* packet_frustum)
{
    const unsigned int top_left = 0;
    const unsigned int top_right = packet_w / 4 - 1;
    const unsigned int bottom_left = (packet_h - 1) * packet_w / 4;
    const unsigned int bottom_right = packet_h * packet_w / 4 - 1;

    /* Build frustum : top */
    COMPUTE_PLANE(packets[top_right].dir + 12,
                  packets[top_left].dir,
                  packets[top_left].org,
                  packet_frustum);
    /* Left */
    COMPUTE_PLANE(packets[top_left].dir,
                  packets[bottom_left].dir,
                  packets[top_left].org,
                  packet_frustum + 4);
    /* Bottom */
    COMPUTE_PLANE(packets[bottom_left].dir,
                  packets[bottom_right].dir + 12,
                  packets[bottom_left].org,
                  packet_frustum + 8);
    /* Right */
    COMPUTE_PLANE(packets[bottom_right].dir + 12,
                  packets[top_right].dir + 12,
                  packets[top_right].org + 12,
                  packet_frustum + 12);
    /* Near & far */
    vector4_copy(near_plane, packet_frustum + 16);
    vector4_copy(far_plane, packet_frustum + 20);
}

void build_packets_shadows(ray_packet* packets,
                           unsigned int num_packets,
                           const ray_packet* prev_packets,
                           const packet_hit* hits,
                           const float* light_pos)
{
    for (unsigned int i = 0; i < num_packets; i++) {
        for (int j = 0; j < 4; j++) {
            if (hits[i].intr[j] >= 0) {
                /* If the ray has hit, we can build a shadow ray */
                float pos[4];
                vector3_scale(prev_packets[i].dir + j * 4, hits[i].t[j], pos);
                vector3_add(pos, prev_packets[i].org + j * 4, pos);

                vector3_copy(light_pos, packets[i].org + j * 4);
                packets[i].org[j * 4 + 3] = 1.0f;

                vector3_subtract(pos, light_pos, packets[i].dir + j * 4);
                packets[i].dir[j * 4 + 3] = 0.0f;

                packets[i].inv_dir[j * 4 + 0] = 1.0f / packets[i].dir[j * 4 + 0];
                packets[i].inv_dir[j * 4 + 1] = 1.0f / packets[i].dir[j * 4 + 1];
                packets[i].inv_dir[j * 4 + 2] = 1.0f / packets[i].dir[j * 4 + 2];
                packets[i].inv_dir[j * 4 + 3] = FLT_MAX;
            } else {
                /* Otherwise we build an empty ray */
                vector3_copy(light_pos, packets[i].org + j * 4);
                packets[i].org[0] = 1.0f;
                packets[i].dir[j * 4 + 0] = 0.0f;
                packets[i].dir[j * 4 + 1] = 0.0f;
                packets[i].dir[j * 4 + 2] = 0.0f;
                packets[i].dir[j * 4 + 3] = 0.0f;

                packets[i].inv_dir[j * 4 + 0] = FLT_MAX;
                packets[i].inv_dir[j * 4 + 1] = FLT_MAX;
                packets[i].inv_dir[j * 4 + 2] = FLT_MAX;
                packets[i].inv_dir[j * 4 + 3] = FLT_MAX;
            }
        }
    }
}

void build_frustum_shadows(const ray_packet* packets,
						   unsigned int num_packets,
                           const float* light_pos,
                           float* packet_frustum)
{
    /* Builds a bounding box for a shadow packet and store
     * the resulting 6 planes the packet frustum */
    float min[4], max[4];

    vector4_copy(light_pos, min);
    vector4_copy(light_pos, max);

    for (unsigned int i = 0; i < num_packets; i++) {
        for (int j = 0; j < 4; j++) {
            float pos[4];
            vector3_add(packets[i].dir + j * 4, packets[i].org + j * 4, pos);
            vector3_min(min, pos, min);
            vector3_max(max, pos, max);
        }
    }

    for (int i = 0; i < 6; i += 2) {
        int j = i / 2;
        packet_frustum[i * 4 + 0] = 0.0f;
        packet_frustum[i * 4 + 1] = 0.0f;
        packet_frustum[i * 4 + 2] = 0.0f;
        packet_frustum[i * 4 + 3] = min[j];
        packet_frustum[i * 4 + j] = -1.0f;

        packet_frustum[(i + 1) * 4 + 0] = 0.0f;
        packet_frustum[(i + 1) * 4 + 1] = 0.0f;
        packet_frustum[(i + 1) * 4 + 2] = 0.0f;
        packet_frustum[(i + 1) * 4 + 3] = -max[j];
        packet_frustum[(i + 1) * 4 + j] = 1.0f;
    }
}
