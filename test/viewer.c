#include "bvh.h"
#include "intr.h"
#include "view.h"
#include "linalg.h"
#include "shader.h"
#include "mem.h"
#include "obj_loader.h"
#include "tga_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <math.h>
#include <SDL/SDL.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define VIEW_WIDTH 512         /* View window width (in pixels) */
#define VIEW_HEIGHT 512        /* View window height (in pixels) */
#define PACKET_WIDTH 8         /* Number of horizontal rays per bundle */
#define PACKET_HEIGHT 8        /* Number of vertical rays per bundle */

#define NUM_PACKETS (PACKET_WIDTH * PACKET_HEIGHT / 4)

typedef void (*shader_func_ptr) (const packet_hit*, const ray_packet*,
                                 const packet_hit*, const ray_packet*,
                                 unsigned int, const shader_info*,
                                 rgb* result);

typedef struct
{
    char name[TEX_NAME];
    unsigned int w, h;
    rgb* ptr;
} shader_texture;

void check(bvh_node* nodes, int* orgs, unsigned int idx)
{
    /* Checks the presence of a cycle in the tree */
    if (orgs[idx] != 0)
        printf("warning : invalid BVH created\n");

    orgs[idx] = 1;
    if (nodes[idx].num_tris == 0) {
        check(nodes, orgs, nodes[idx].node_data.child);
        check(nodes, orgs, nodes[idx].node_data.child + 1);
    }
}

void display(bvh_node* nodes, unsigned int d, unsigned int idx)
{
    for (unsigned int i = 0; i < d; i++) {
        putchar(' ');
    }
    printf("%u : %d\n", idx, nodes[idx].num_tris);

    if (nodes[idx].num_tris == 0) {
        display(nodes, d + 1, nodes[idx].node_data.child);
        display(nodes, d + 1, nodes[idx].node_data.child + 1);
    }
}

void intr_all_tris(const precomp_tri* tris, unsigned int num_tris, const ray_packet* packet, packet_hit* hit)
{
    /* This is a DEBUG function to display an image without a BVH */
    packet_hit SSE_ALIGN(hit2);
 
    hit->t[0] = FLT_MAX;
    hit->t[1] = FLT_MAX;
    hit->t[2] = FLT_MAX;
    hit->t[3] = FLT_MAX;

    hit->intr[0] = -1;
    hit->intr[1] = -1;
    hit->intr[2] = -1;
    hit->intr[3] = -1;

    for (unsigned int t = 0; t < num_tris; t++) {
        hit2.t[0] = hit->t[0];
        hit2.t[1] = hit->t[1];
        hit2.t[2] = hit->t[2];
        hit2.t[3] = hit->t[3];

        intr_packet_tri(packet, tris + t, &hit2);

        for (int i = 0; i < 4; i++) {     
            if (hit2.intr[i] && hit2.t[i] < hit->t[i]) {
                hit->u[i] = hit2.u[i];
                hit->v[i] = hit2.v[i];
                hit->t[i] = hit2.t[i];
                hit->intr[i] = t;
            }
        }
    }
}

void render_image(SDL_Surface* screen,
                  const bvh_obj* bvh,
                  const precomp_tri* tris,
                  unsigned int num_tris,
                  const view_info* view,
                  const shader_info* shader,
                  const shader_func_ptr shader_func)
{
	SDL_LockSurface(screen);

    #pragma omp parallel for schedule(dynamic)
    for (int y = 0; y < VIEW_HEIGHT; y += PACKET_HEIGHT) {
        for (unsigned int x = 0; x < VIEW_WIDTH; x += PACKET_WIDTH) {
            ray_packet SSE_ALIGN(packets[NUM_PACKETS]);
            packet_hit SSE_ALIGN(hits[NUM_PACKETS]);
            float SSE_ALIGN(frustum[6 * 4]);

            /* Build primary rays packets */
            build_packets_persp(view, packets, x , y,
                                PACKET_WIDTH, PACKET_HEIGHT,
                                VIEW_WIDTH, VIEW_HEIGHT);
            build_frustum_persp(packets, PACKET_WIDTH, PACKET_HEIGHT,
                                view->near, view->far, frustum);

            closest_intr_bvh(packets, NUM_PACKETS, frustum, bvh, tris, hits);

            ray_packet SSE_ALIGN(shadow_packets[NUM_PACKETS * shader->num_lights]);
            packet_hit SSE_ALIGN(shadow_hits[NUM_PACKETS * shader->num_lights]);

            /* Build shadow rays packets */
            for (unsigned int l = 0; l < shader->num_lights; l++) {
                build_packets_shadows(shadow_packets + l * NUM_PACKETS, NUM_PACKETS, packets, hits, shader->lights[l].pos);
                build_frustum_shadows(shadow_packets + l * NUM_PACKETS, NUM_PACKETS, shader->lights[l].pos, frustum);

                first_intr_bvh(shadow_packets + l * NUM_PACKETS, NUM_PACKETS, frustum, bvh, tris, shadow_hits + l * NUM_PACKETS, hits);
            }

            /* Shade pixels */
            rgb pixels[NUM_PACKETS * 4];
            shader_func(hits, packets, shadow_hits, shadow_packets, NUM_PACKETS, shader, pixels);

            /* Copy them to screen */
            for (unsigned int i = 0; i < PACKET_HEIGHT; i++) {
                for (unsigned int j = 0; j < PACKET_WIDTH / 4; j++) {
                    for (unsigned int k = 0; k < 4; k++) {
                        unsigned char* p = (unsigned char*)screen->pixels + screen->pitch * (y + i) + (x + j * 4 + k) * 4;
                        const rgb* color = pixels + (i * PACKET_WIDTH / 4 + j) * 4 + k;
                        p[0] = color->b;
                        p[1] = color->g;
                        p[2] = color->r;
                        p[3] = color->a;
                    }
                }
            }
        }
    }

    SDL_UnlockSurface(screen);
}

int handle_events(float* up, float* eye, float* pos, shader_info* shader, shader_func_ptr* shader_func)
{
    SDL_Event event;
    const float t_speed = 1.0f;
    const float r_speed = 1.0f;
    const float c_speed = 1.0f;
    const float x_sensitivity = 0.001f;
    const float y_sensitivity = 0.001f;
    float tx = 0.0f;
    float ty = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;

    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
		            case SDLK_UP:
                        ty += t_speed;
                        break;
                    case SDLK_DOWN:
                        ty -= t_speed;
                        break;
                    case SDLK_LEFT:
                        tx += t_speed;
                        break;
                    case SDLK_RIGHT:
	                    tx -= t_speed;
                        break;
                    case SDLK_ESCAPE:
                        return 1;
                    case SDLK_s:
                        if (*shader_func == cel_shader)
                            *shader_func = phong_shader;
                        else
                            *shader_func = cel_shader;
                        break;
                    case SDLK_u:
                        shader->lights[0].pos[0] += t_speed;
                        break;
                    case SDLK_j:
                        shader->lights[0].pos[0] -= t_speed;
                        break;
                    case SDLK_i:
                        shader->lights[0].pos[1] += t_speed;
                        break;
                    case SDLK_k:
                        shader->lights[0].pos[1] -= t_speed;
                        break;
                    case SDLK_o:
                        shader->lights[0].pos[2] += t_speed;
                        break;
                    case SDLK_l:
                        shader->lights[0].pos[2] -= t_speed;
                        break;
                    default:
                        break;
                }
                break;
	        case SDL_MOUSEBUTTONDOWN:
                switch(event.button.button) {
		            case SDL_BUTTON_WHEELUP:
        	            ty += c_speed;
                        break;
                    case SDL_BUTTON_WHEELDOWN:
                        ty -= c_speed;
                        break;
                    default:
                        break;
                }
            case SDL_MOUSEMOTION:
                rx = -r_speed * event.motion.xrel * x_sensitivity;
                ry = r_speed * event.motion.yrel * y_sensitivity;
                break;
            case SDL_QUIT:
                return 1;
            default:
                break;
        }
    }
    
    if (fabs(rx) < 1 && fabs(ry) < 1)
        update_view(tx, ty, rx, ry, up, eye, pos);

    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("usage : %s file.obj [file.mtl]\n", argv[0]);
        return 1;
    }

    /* Load model */
    obj_model model;
    if (!load_obj(&model, argv[1])) {
        printf("error : cannot open input file\n");
        return 1;
    }

    if (model.num_faces == 0)
    {
        printf("error : no faces could be extracted from the model\n");
        return 1;
    }

    printf("faces : %d\nvertices : %d\nnormals : %d\ntexcoords : %d\n",
        model.num_faces, model.num_vertices,
        model.num_normals, model.num_texcoords);

    if (model.num_normals == 0) {
        printf("error : this model has no normals\n");
        return 1;    
    }

    printf("materials : %d\n", model.num_materials);
    for (unsigned int i = 0, j = 0;
         i < model.num_materials;
         i++, j += sizeof(char) * MAT_NAME) {
        printf("%d : %s\n", i, model.materials + j);
    }

    /* Load materials file */
    obj_mtl mtl;
    char mtl_file[256];
        
    if (argc < 3) {
        char* ptr = strrchr(argv[1], '.');
        if (ptr) *ptr = '\0';
        sprintf(mtl_file, "%s.mtl", argv[1]);
        if (ptr) *ptr = '.';
    } else {
        strcpy(mtl_file, argv[2]);    
    }

    if (!load_mtl(&mtl, mtl_file)) {
        printf("error : cannot open materials file\n");
        return 1;
    }

    if (mtl.num_materials < model.num_materials) {
        printf("error : numbers of materials do not match\n");
        return 1;
    }

    /* Setup shader */
    shader_info shader;
    shader_texture textures[mtl.num_materials];
    memset(textures, 0, sizeof(shader_texture) * mtl.num_materials);
    unsigned int num_textures = 0;

    shader.materials = malloc(sizeof(material) * mtl.num_materials);
    for (unsigned int i = 0; i < mtl.num_materials; i++) {
        vector3_copy(mtl.materials[i].diffuse, shader.materials[i].diffuse);
        shader.materials[i].diffuse[3] = 0.0f;
        vector3_copy(mtl.materials[i].specular, shader.materials[i].specular);
        shader.materials[i].specular[3] = 0.0f;
        vector3_copy(mtl.materials[i].ambient, shader.materials[i].ambient);
        shader.materials[i].ambient[3] = 0.0f;

        shader.materials[i].alpha = mtl.materials[i].alpha;

        /* Load TGA texture */
        shader.materials[i].texture = NULL;
        shader.materials[i].tex_mode = TEX_REPEAT;
        if (strlen(mtl.materials[i].tex_name) > 0) {
            /* Changes Windows slashes to UNIX slashes */
            for (char* ptr = mtl.materials[i].tex_name; *ptr != 0; ptr++) {
                if (*ptr == '\\') *ptr = '/';
            }

            /* Try to find texture */
            unsigned int tex_index;
            for (tex_index = 0; tex_index < num_textures; tex_index++) {
                if (!strcmp(textures[tex_index].name, mtl.materials[i].tex_name))
                    break;
            }

            if (tex_index >= num_textures) {
                rgb* texture;
                textures[num_textures].ptr = NULL;

                /* Try to load texture */
                if (load_tga(&texture,
                             &textures[num_textures].w,
                             &textures[num_textures].h,
                             mtl.materials[i].tex_name)) {
                    printf("texture  : \"%s\" %dx%d pixels\n",
                           mtl.materials[i].tex_name,
                           textures[num_textures].w,
                           textures[num_textures].h);

                    textures[num_textures].ptr = texture;
                }
                else
                    printf("warning : cannot load texture \"%s\"\n", mtl.materials[i].tex_name);

                strcpy(textures[num_textures].name, mtl.materials[i].tex_name);
                tex_index = num_textures;
                num_textures++;
            }

            shader.materials[i].tex_w = textures[tex_index].w;
            shader.materials[i].tex_h = textures[tex_index].h;
            shader.materials[i].texture = textures[tex_index].ptr;
        }
    }

    shader.vertices = aligned_malloc(sizeof(float) * 4 * model.num_vertices, 16);
    for (unsigned int i = 0; i < model.num_vertices; i++) {
        shader.vertices[i * 4 + 0] = model.vertices[i * 3 + 0];
        shader.vertices[i * 4 + 1] = model.vertices[i * 3 + 1];
        shader.vertices[i * 4 + 2] = model.vertices[i * 3 + 2];
        shader.vertices[i * 4 + 3] = 1.0f;
    }

    shader.normals = aligned_malloc(sizeof(float) * 4 * model.num_normals, 16);
    for (unsigned int i = 0; i < model.num_normals; i++) {
        shader.normals[i * 4 + 0] = model.normals[i * 3 + 0];
        shader.normals[i * 4 + 1] = model.normals[i * 3 + 1];
        shader.normals[i * 4 + 2] = model.normals[i * 3 + 2];
        shader.normals[i * 4 + 3] = 0.0f;
    }

    shader.texcoords = aligned_malloc(sizeof(float) * 2 * model.num_texcoords, 16);
    for (unsigned int i = 0; i < model.num_texcoords; i++) {
        shader.texcoords[i * 2 + 0] = model.texcoords[i * 2 + 0];
        shader.texcoords[i * 2 + 1] = model.texcoords[i * 2 + 1];
    }

    shader.faces = malloc(sizeof(face) * model.num_faces);
    for (unsigned int i = 0; i < model.num_faces; i++) {
        /* Indices in OBJ format begin at 1 */
        shader.faces[i].vertices[0] = model.faces[i].vertices[0] - 1;
        shader.faces[i].vertices[1] = model.faces[i].vertices[1] - 1;
        shader.faces[i].vertices[2] = model.faces[i].vertices[2] - 1;

        shader.faces[i].normals[0] = model.faces[i].normals[0] - 1;
        shader.faces[i].normals[1] = model.faces[i].normals[1] - 1;
        shader.faces[i].normals[2] = model.faces[i].normals[2] - 1;

        shader.faces[i].texcoords[0] = model.faces[i].texcoords[0] - 1;
        shader.faces[i].texcoords[1] = model.faces[i].texcoords[1] - 1;
        shader.faces[i].texcoords[2] = model.faces[i].texcoords[2] - 1;

        /* Associate with the right material */
        const char* mat_name = model.materials + MAT_NAME * model.faces[i].mat_id;
        unsigned int idx;
        for (idx = 0; idx < mtl.num_materials; idx++) {
            if (!strcmp(mtl.materials[idx].mat_name, mat_name)) break;        
        }

        if (idx >= mtl.num_materials) {
            idx = 0;
            printf("warning : material \"%s\" not found using 0 instead\n", mat_name);
        }

        shader.faces[i].material = idx;
    }

    light SSE_ALIGN(shader_lights[1]) =
    {
        {
            /* pos */
            {5.0f, 10.0f, 5.0f, 1.0f},
            /* diffuse */
            {0.8f, 0.8f, 0.8f, 0.8f},
            /* specular */
            {0.3f, 0.3f, 0.3f, 0.3f}
        }
    };

    shader.lights = shader_lights;
    shader.num_lights = 1;

    shader.ambient[0] = 0.2f;
    shader.ambient[1] = 0.2f;
    shader.ambient[2] = 0.2f;
    shader.ambient[3] = 0.2f;

    shader.bg_color.r = 53;
    shader.bg_color.g = 93;
    shader.bg_color.b = 144;
    shader.bg_color.a = 0;

    unsigned int* indices = malloc(sizeof(int) * 3 * model.num_faces);
    for (unsigned int i = 0; i < model.num_faces; i++) {
        /* Indices in OBJ format begin at 1 */
        indices[i * 3 + 0] = model.faces[i].vertices[0] - 1;
        indices[i * 3 + 1] = model.faces[i].vertices[1] - 1;
        indices[i * 3 + 2] = model.faces[i].vertices[2] - 1;
    }

    /* Build a BVH */
    bvh_obj bvh;
    clock_t ck = clock();
    unsigned int num_nodes = build_bvh(indices, model.num_faces, shader.vertices, &bvh);
    printf("bvh build : %ld ms\n", (clock() - ck) * 1000 / CLOCKS_PER_SEC);
    printf("nodes : %u\ndepth : %u\n", num_nodes, bvh.depth);

    /* Check the tree */
    int origins[num_nodes];
    memset(origins, 0, sizeof(int) * num_nodes);
    check(bvh.root, origins, 0);

    /* Display small trees */
    if (num_nodes < 1000) {
        display(bvh.root, 0, 0);
    }

    /* Precompute triangles */
	ck = clock();
    precomp_tri* tris = aligned_malloc(sizeof(precomp_tri) * model.num_faces, 16);
    precompute(shader.vertices, indices, tris, model.num_faces);
	printf("precompute : %ld ms\n", (clock() - ck) * 1000 / CLOCKS_PER_SEC);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("error : cannot initialise SDL (%s)\n", SDL_GetError());
        return 1;
    }

    SDL_WM_SetCaption("TheDreamMachine viewer", NULL);

    SDL_Surface* screen = SDL_SetVideoMode(VIEW_WIDTH, VIEW_HEIGHT, 32, 0);
    if (!screen) {
        printf("error : cannot set video mode\n");
        return 1;
    }

    view_info view;   

    float SSE_ALIGN(eye[4]) = {0.0f, 25.0f, -25.0f, 0};
    float SSE_ALIGN(pos[4]) = {0.0f, 0.0f, 0.0f, 1.0f};
    float SSE_ALIGN(up[4]) = {0.0f, -1.0f, 0.0f, 0.0f};

    shader_func_ptr shader_func = phong_shader;

    int done = 0;
    unsigned int frames = 0;

    SDL_ShowCursor(0);
    SDL_WM_GrabInput(SDL_GRAB_ON);
    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

    unsigned int ticks = SDL_GetTicks();
    while (!done) {
        /* Setup view */
        setup_view_persp(&view,
                         eye, pos, up,
                         60.0f, (float)VIEW_WIDTH / (float)VIEW_HEIGHT,
                         0.1f, 1000.0f);

        if (SDL_GetTicks() - ticks >= 5000) {
            printf("fps : %f\n", (float)frames / 5.0f);
            frames = 0;
            ticks = SDL_GetTicks();
        }
        render_image(screen, &bvh, tris, model.num_faces, &view, &shader, shader_func);
        done = handle_events(up, eye, pos, &shader, &shader_func);
        SDL_Flip(screen);
        frames++;
    }

    SDL_WM_GrabInput(SDL_GRAB_OFF);
    SDL_Quit();

    /* Release memory */
    for (unsigned int i = 0; i < num_textures; i++) {
        if (textures[i].ptr)
            destroy_tga(textures[i].ptr);
    }

    free(shader.materials);
    aligned_free(shader.vertices);
    aligned_free(shader.normals);
    aligned_free(shader.texcoords);
    free(shader.faces);

    destroy_bvh(&bvh);
    destroy_obj(&model);
    destroy_mtl(&mtl);

    aligned_free(tris);
    free(indices);

    return 0;
}

