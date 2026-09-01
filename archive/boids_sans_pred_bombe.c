#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define WIDTH 800
#define HEIGHT 800
#define N_BOIDS 1000

#define V_LIM 0.5
#define R_COHESION 80
#define R_ALIGNEMENT 40
#define R_SEPARATION 10
#define W_COH 0.002
#define W_ALIGN 0.004
#define W_SEP 0.02
#define W_COH_MIN 0.0005
#define W_COH_MAX 0.01
#define W_ALIGN_MIN 0.0005
#define W_ALIGN_MAX 0.02
#define W_SEP_MIN 0.005
#define W_SEP_MAX 0.05
#define V_LIM_MIN 0.2
#define V_LIM_MAX 1.2

#define TAILLE_HASH R_COHESION
#define NB_CASE_X (WIDTH / TAILLE_HASH)
#define NB_CASE_Y (HEIGHT / TAILLE_HASH)

#define FPS_VISE 60.0

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FX_SHIFT 16
#define FX_ONE (1 << FX_SHIFT)
typedef int32_t fx;

static inline fx fx_from_double(double d){ return (fx)(d * FX_ONE); }
static inline fx fx_mul(fx a, fx b){ return (fx)(((int64_t)a * b) >> FX_SHIFT); }
static inline fx fx_div(fx a, fx b){ return (fx)(((int64_t)a << FX_SHIFT) / b); }
static inline fx fx_norm2(fx x, fx y){ return (fx)(((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT); }
static inline fx fx_sqrt(fx a){
    double d = (double)a / (double)FX_ONE;
    return fx_from_double(sqrt(d));
}

typedef struct {
    int id;
    fx x, y;
    fx vx, vy;
    fx ax, ay;
    int case_x, case_y;
} Boid;

typedef struct hashed{
    int id;
    struct hashed* next;
} hashed;

Boid boids[N_BOIDS];
hashed* grille[NB_CASE_X][NB_CASE_Y] = {0};

fx fx_r_coh2, fx_r_align2, fx_r_sep2;
fx fx_w_coh, fx_w_align, fx_w_sep;
fx fx_v_lim2;
fx fx_w_ecran, fx_h_ecran, fx_half_w, fx_half_h;
double w_coh = W_COH, w_align = W_ALIGN, w_sep = W_SEP;
double v_lim = V_LIM;

static inline void ajouter_boid_a_case(int cx, int cy, int id){
    hashed* c = (hashed*)malloc(sizeof(hashed));
    c->id = id;
    c->next = grille[cx][cy];
    grille[cx][cy] = c;
}

static inline void supprimer_boid_de_case(int cx, int cy, int id){
    hashed** cur = &grille[cx][cy];
    while (*cur){
        if ((*cur)->id == id){
            hashed *victim = *cur;
            *cur = victim->next;
            free(victim);
            break;
        }
        cur = &(*cur)->next;
    }
}

static inline fx fx_rand_range(double min, double max){
    double r = (double)(rand() % 10000) / 10000.0;
    return fx_from_double(min + r * (max - min));
}

static inline double rand_between(double min, double max){
    double r = (double)(rand() % 10000) / 10000.0;
    return min + r * (max - min);
}

static inline void ajouter_accel_dir(fx dx, fx dy, fx coef, fx* ax, fx* ay){
    fx n = fx_sqrt(fx_norm2(dx, dy));
    if (n == 0) return;
    fx ux = fx_div(dx, n);
    fx uy = fx_div(dy, n);
    *ax += fx_mul(coef, ux);
    *ay += fx_mul(coef, uy);
}

static void rafraichir_forces(){
    fx_w_coh = fx_from_double(w_coh);
    fx_w_align = fx_from_double(w_align);
    fx_w_sep = fx_from_double(w_sep);
    fx_v_lim2 = fx_mul(fx_from_double(v_lim), fx_from_double(v_lim));
}

static void randomise_forces(){
    w_coh = rand_between(W_COH_MIN, W_COH_MAX);
    w_align = rand_between(W_ALIGN_MIN, W_ALIGN_MAX);
    w_sep = rand_between(W_SEP_MIN, W_SEP_MAX);
    v_lim = rand_between(V_LIM_MIN, V_LIM_MAX);
    rafraichir_forces();
}

static void reynolds(Boid *b, fx* axs, fx* ays){
    fx Gx = 0, Gy = 0;
    fx Sx = 0, Sy = 0;
    fx vx_ali = 0, vy_ali = 0;
    int nG = 1, nA = 0, nS = 0;

    for (int dx = -1; dx <= 1; dx++){
        for (int dy = -1; dy <= 1; dy++){
            int case_x = (b->case_x + dx + NB_CASE_X) % NB_CASE_X;
            int case_y = (b->case_y + dy + NB_CASE_Y) % NB_CASE_Y;

            for(hashed* h = grille[case_x][case_y]; h; h = h->next){
                Boid* b1 = &boids[h->id];
                if (b1 == b) continue;

                fx rx = b1->x - b->x;
                if (rx > fx_half_w) rx -= fx_w_ecran;
                if (rx < -fx_half_w) rx += fx_w_ecran;

                fx ry = b1->y - b->y;
                if (ry > fx_half_h) ry -= fx_h_ecran;
                if (ry < -fx_half_h) ry += fx_h_ecran;

                fx dist = fx_norm2(rx, ry);

                if(dist <= fx_r_coh2){
                    Gx += rx;
                    Gy += ry;
                    nG += 1;
                    if(dist <= fx_r_align2){
                        vx_ali += b1->vx;
                        vy_ali += b1->vy;
                        nA += 1;
                        if(dist <= fx_r_sep2){
                            Sx -= rx;
                            Sy -= ry;
                            nS += 1;
                        }
                    }
                }
            }
        }
    }
    fx ax_coh = 0, ay_coh = 0, ax_sep = 0, ay_sep = 0, ax_ali = 0, ay_ali = 0;

    if (nS != 0){
        Sx /= nS;
        Sy /= nS;
        ajouter_accel_dir(Sx, Sy, fx_w_sep, &ax_sep, &ay_sep);
    }

    Gx /= nG;
    Gy /= nG;
    ajouter_accel_dir(Gx, Gy, fx_w_coh, &ax_coh, &ay_coh);
    if (nA != 0){
        vx_ali /= nA;
        vy_ali /= nA;
        fx norme = fx_sqrt(fx_norm2(vx_ali, vy_ali));
        if (norme != 0){
            fx ux = fx_div(vx_ali, norme);
            fx uy = fx_div(vy_ali, norme);
            ax_ali = fx_mul(fx_w_align, ux);
            ay_ali = fx_mul(fx_w_align, uy);
        }
    }
    *axs = ax_ali + ax_coh + ax_sep;
    *ays = ay_ali + ay_coh + ay_sep;
}

static void update(Boid* b){
    b->vx += b->ax;
    b->vy += b->ay;

    fx v2 = fx_norm2(b->vx, b->vy);
    if (v2 > fx_v_lim2) {
        fx scale = fx_div(fx_from_double(v_lim), fx_sqrt(v2));
        b->vx = fx_mul(b->vx, scale);
        b->vy = fx_mul(b->vy, scale);
    }

    b->x += b->vx;
    b->y += b->vy;

    if (b->x < 0) b->x += fx_w_ecran;
    if (b->x >= fx_w_ecran) b->x -= fx_w_ecran;
    if (b->y < 0) b->y += fx_h_ecran;
    if (b->y >= fx_h_ecran) b->y -= fx_h_ecran;

    int ncx = (b->x >> FX_SHIFT) / TAILLE_HASH;
    int ncy = (b->y >> FX_SHIFT) / TAILLE_HASH;

    if (ncx != b->case_x || ncy != b->case_y) {
        supprimer_boid_de_case(b->case_x, b->case_y, b->id);
        b->case_x = ncx;
        b->case_y = ncy;
        ajouter_boid_a_case(ncx, ncy, b->id);
    }
}

static void init_constants(){
    fx r_coh = fx_from_double(R_COHESION);
    fx r_ali = fx_from_double(R_ALIGNEMENT);
    fx r_sep = fx_from_double(R_SEPARATION);
    fx_r_coh2 = fx_mul(r_coh, r_coh);
    fx_r_align2 = fx_mul(r_ali, r_ali);
    fx_r_sep2 = fx_mul(r_sep, r_sep);

    w_coh = W_COH;
    w_align = W_ALIGN;
    w_sep = W_SEP;
    v_lim = V_LIM;
    rafraichir_forces();

    fx_w_ecran = fx_from_double(WIDTH);
    fx_h_ecran = fx_from_double(HEIGHT);
    fx_half_w = fx_w_ecran / 2;
    fx_half_h = fx_h_ecran / 2;
}

static void draw_arrow(SDL_Renderer* r, fx x, fx y, fx vx, fx vy, double size){
    double px = (double)x / FX_ONE;
    double py = (double)y / FX_ONE;
    double heading = atan2((double)vy, (double)vx);
    double tip_x = px + cos(heading) * size;
    double tip_y = py + sin(heading) * size;
    double left_x = px + cos(heading + 2.5) * size * 0.6;
    double left_y = py + sin(heading + 2.5) * size * 0.6;
    double right_x = px + cos(heading - 2.5) * size * 0.6;
    double right_y = py + sin(heading - 2.5) * size * 0.6;

    SDL_Point pts[4] = {
        {(int)tip_x, (int)tip_y},
        {(int)left_x, (int)left_y},
        {(int)right_x, (int)right_y},
        {(int)tip_x, (int)tip_y}
    };
    SDL_RenderDrawLines(r, pts, 4);
}

static void draw_gauge(SDL_Renderer* r, TTF_Font* font, int x, int y, double val, double vmin, double vmax, const char* label, SDL_Color color, SDL_Color back){
    int w = 160;
    int h = 14;
    double t = (val - vmin) / (vmax - vmin);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    SDL_Rect bg = {x, y, w, h};
    SDL_SetRenderDrawColor(r, back.r, back.g, back.b, back.a);
    SDL_RenderFillRect(r, &bg);
    SDL_Rect fg = {x, y, (int)(w * t), h};
    SDL_SetRenderDrawColor(r, color.r, color.g, color.b, color.a);
    SDL_RenderFillRect(r, &fg);
    char buf[64];
    snprintf(buf, sizeof(buf), "%s: %.4f", label, val);
    SDL_Surface* surf = TTF_RenderText_Solid(font, buf, (SDL_Color){255,255,255,255});
    if (surf){
        SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
        SDL_FreeSurface(surf);
        if (tex){
            SDL_Rect dst = {x + w + 8, y - 2, 0, 0};
            SDL_QueryTexture(tex, NULL, NULL, &dst.w, &dst.h);
            SDL_RenderCopy(r, tex, NULL, &dst);
            SDL_DestroyTexture(tex);
        }
    }
}

int main() {
    init_constants();

    SDL_Window* w = SDL_CreateWindow(
        "Boids simples",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );

    SDL_Renderer* r = SDL_CreateRenderer(w, -1, SDL_RENDERER_ACCELERATED);

    TTF_Init();
    TTF_Font* font = TTF_OpenFont("Open_Sans/OpenSans-VariableFont_wdth,wght.ttf", 18);

    SDL_Color fps_color = {255, 255, 255, 255};

    for (int i = 0; i < N_BOIDS; i++) {
        boids[i].id = i;
        boids[i].x = fx_from_double(rand() % WIDTH);
        boids[i].y = fx_from_double(rand() % HEIGHT);
        boids[i].vx = fx_rand_range(-0.2, 0.2);
        boids[i].vy = fx_rand_range(-0.2, 0.2);
        boids[i].ax = boids[i].ay = 0;

        boids[i].case_x = (boids[i].x >> FX_SHIFT) / TAILLE_HASH;
        boids[i].case_y = (boids[i].y >> FX_SHIFT) / TAILLE_HASH;
        ajouter_boid_a_case(boids[i].case_x, boids[i].case_y, i);
    }

    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 debut = SDL_GetPerformanceCounter();
    double accum_random = 0.0;
    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e))
            if (e.type == SDL_QUIT) running = 0;
            else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r){
                randomise_forces();
            }

        SDL_SetRenderDrawColor(r,135,206,235,255);
        SDL_RenderClear(r);

        SDL_SetRenderDrawColor(r,255,255,255,255);
        for (int i = 0; i < N_BOIDS; i++) {
            reynolds(&boids[i], &boids[i].ax, &boids[i].ay);
            update(&boids[i]);
            draw_arrow(r, boids[i].x, boids[i].y, boids[i].vx, boids[i].vy, 8.0);
        }

        SDL_Color c_coh = {80, 200, 255, 255};
        SDL_Color c_alig = {120, 255, 120, 255};
        SDL_Color c_sep = {255, 170, 80, 255};
        SDL_Color c_vit = {255, 80, 150, 255};
        SDL_Color c_bg = {40, 40, 40, 200};
        int gy = 40;
        draw_gauge(r, font, 20, gy, w_coh, W_COH_MIN, W_COH_MAX, "cohesion", c_coh, c_bg); gy += 22;
        draw_gauge(r, font, 20, gy, w_align, W_ALIGN_MIN, W_ALIGN_MAX, "align", c_alig, c_bg); gy += 22;
        draw_gauge(r, font, 20, gy, w_sep, W_SEP_MIN, W_SEP_MAX, "separation", c_sep, c_bg); gy += 22;
        draw_gauge(r, font, 20, gy, v_lim, V_LIM_MIN, V_LIM_MAX, "v_lim", c_vit, c_bg);

        Uint64 fin = SDL_GetPerformanceCounter();
        Uint64 elapsed_ticks = fin - debut;
        Uint64 target_ticks = (Uint64)((double)freq / FPS_VISE);
        if (elapsed_ticks < target_ticks){
            Uint64 remaining = target_ticks - elapsed_ticks;
            Uint32 delay_ms = (Uint32)(remaining * 1000 / freq);
            if (delay_ms > 0){
                SDL_Delay(delay_ms);
            }
            while ((SDL_GetPerformanceCounter() - fin) < remaining){}
            fin = SDL_GetPerformanceCounter();
            elapsed_ticks = fin - debut;
        }

        double delta = (double)elapsed_ticks/(double)(freq);
        double fps = delta > 0 ? 1.0/delta : 0.0;
        accum_random += delta;
        if (accum_random >= 5.0){
            randomise_forces();
            accum_random = 0.0;
        }
        char texte_fps[32];
        snprintf(texte_fps, sizeof(texte_fps), "FPS: %.2f", fps);

        SDL_Surface* surfaceTexte = TTF_RenderText_Solid(font, texte_fps, fps_color);
        if (!surfaceTexte){
            fprintf(stderr, "TTF_RenderText_Solid error: %s\n", TTF_GetError());
        } else {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(r, surfaceTexte);
            SDL_FreeSurface(surfaceTexte);
            if (texture){
                SDL_Rect txt = {10, 10, 0, 0};
                SDL_QueryTexture(texture, NULL, NULL, &txt.w, &txt.h);
                SDL_RenderCopy(r, texture, NULL, &txt);
                SDL_DestroyTexture(texture);
            }
        }

        SDL_RenderPresent(r);
        debut = fin;
    }

    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(r);
    SDL_DestroyWindow(w);
    SDL_Quit();
    return 0;
}
