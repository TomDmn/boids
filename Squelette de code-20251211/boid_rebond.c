#include <SDL2/SDL.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* ================= CONFIG ================= */

#define WIDTH 1500
#define HEIGHT 900
#define N_BOIDS 200

/* rayons (pixels) */
#define R_SEP     8
#define R_ALIGN   150
#define R_COH     200

/* poids */
#define W_SEP     2.0
#define W_ALIGN   0.045
#define W_COH     0.4

/* dynamique */
#define ACC_SCALE 0.08
#define MAX_SPEED 1.5



#define HASH_SIZE R_COH
#define NB_CASE_X ((WIDTH  + HASH_SIZE - 1) / HASH_SIZE)
#define NB_CASE_Y ((HEIGHT + HASH_SIZE - 1) / HASH_SIZE)

/* ================= FIXED POINT ================= */

#define FX_SHIFT 16
#define FX_ONE (1 << FX_SHIFT)
typedef int32_t fx;

static inline fx fx_from_double(double d){ return (fx)(d * FX_ONE); }
static inline fx fx_mul(fx a, fx b){ return (fx)(((int64_t)a * b) >> FX_SHIFT); }
static inline fx fx_div(fx a, fx b){ return (fx)(((int64_t)a << FX_SHIFT) / b); }
static inline fx fx_norm2(fx x, fx y){
    return (fx)(((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT);
}
static inline fx fx_sqrt(fx a){
    double d = (double)a / (double)FX_ONE;
    double r = sqrt(d);
    return fx_from_double(r);
}

/* ================= STRUCTURES ================= */

typedef struct {
    fx x,y,vx,vy,ax,ay;
} Boid;

/* ================= GLOBALS ================= */

Boid boids[N_BOIDS];

fx FX_R_COH2, FX_R_ALIGN2, FX_R_SEP2;
fx FX_W_COH, FX_W_ALIGN, FX_W_SEP;
fx FX_ACC_SCALE, FX_MAX_SPEED2;
fx FX_W, FX_H;

/* ================= INIT ================= */

void init_constants(){
    fx r_coh = fx_from_double(R_COH);
    fx r_align = fx_from_double(R_ALIGN);
    fx r_sep = fx_from_double(R_SEP);
    FX_R_COH2 = fx_mul(r_coh, r_coh);
    FX_R_ALIGN2 = fx_mul(r_align, r_align);
    FX_R_SEP2 = fx_mul(r_sep, r_sep);

    FX_W_COH=fx_from_double(W_COH);
    FX_W_ALIGN=fx_from_double(W_ALIGN);
    FX_W_SEP=fx_from_double(W_SEP);

    FX_ACC_SCALE=fx_from_double(ACC_SCALE);
    fx ms = fx_from_double(MAX_SPEED);
    FX_MAX_SPEED2=fx_mul(ms, ms);

    FX_W=fx_from_double(WIDTH);
    FX_H=fx_from_double(HEIGHT);
}

/* ================= REYNOLDS ================= */
/* déplacement indirect vers un point (fixed-point) */
/* retourne l'accélération ax, ay */

static inline void deplacement_indirect_vers_un_point(int i,fx dx, fx dy,fx C,fx *ax, fx *ay){
    Boid* b=&boids[i];
    fx norm = fx_sqrt(fx_norm2(dx,dy));
    if (norm == 0){
        return;
    }
    fx ux = fx_div(dx, norm);
    fx uy = fx_div(dy, norm);

    *ax += fx_mul(ux, C);
    *ay += fx_mul(uy, C);
}

void reynolds(int i){
    Boid* b=&boids[i];
    fx coh_x=0,coh_y=0,sep_x=0,sep_y=0,ali_x=0,ali_y=0;
    int n_coh=0,n_sep=0,n_ali=0;

    for(int j=0;j<N_BOIDS;j++){
        if(j==i) continue;
        Boid* o=&boids[j];

        fx rx=o->x-b->x;
        fx ry=o->y-b->y;

        fx d2=fx_norm2(rx,ry);

        if(d2<FX_R_COH2){
            coh_x += rx;
            coh_y += ry;
            n_coh++;
            if(d2<FX_R_ALIGN2){
                ali_x+=o->vx;
                ali_y+=o->vy;
                n_ali++;
                if(d2<FX_R_SEP2){
                    sep_x -= rx;
                    sep_y -= ry;
                    n_sep++; 
                }
            }
        }
    }

    b->ax=0; b->ay=0;

    if(n_sep){
        sep_x/=n_sep;
        sep_y/=n_sep;
        deplacement_indirect_vers_un_point(i,sep_x,sep_y,FX_W_SEP,&b->ax, &b->ay);
    }
    if(n_coh){
        coh_x /= n_coh;
        coh_y /= n_coh;
        deplacement_indirect_vers_un_point(i,coh_x,coh_y,FX_W_COH,&b->ax, &b->ay);
    }
    if(n_ali){
        ali_x/=n_ali;
        ali_y/=n_ali;
        fx norm = fx_sqrt(fx_norm2(ali_x,ali_y));
        fx ux = fx_div(ali_x,norm);
        fx uy = fx_div(ali_y,norm);
        b->ax+=fx_mul(ux,FX_W_ALIGN);
        b->ay+=fx_mul(uy,FX_W_ALIGN);
    }
    b->ax=fx_mul(b->ax,FX_ACC_SCALE);
    b->ay=fx_mul(b->ay,FX_ACC_SCALE);
}

/* ================= UPDATE ================= */

void update_boid(int i){
    Boid* b=&boids[i];
    b->vx+=b->ax; b->vy+=b->ay;

    fx v2=fx_norm2(b->vx,b->vy);
    if (v2 > fx_from_double(MAX_SPEED*MAX_SPEED)) {
        fx scale = fx_div(fx_from_double(MAX_SPEED), fx_sqrt(v2));
        b->vx = fx_mul(b->vx, scale);
        b->vy = fx_mul(b->vy, scale);
    }

    b->x+=b->vx; b->y+=b->vy;
    if (b->x < 0) {
        b->x = 0;
        b->vx = -b->vx;
    }
    else if (b->x >= FX_W) {
        b->x = FX_W - FX_ONE;  
        b->vx = -b->vx;
    }

    if (b->y < 0) {
        b->y = 0;
        b->vy = -b->vy;
    }
    else if (b->y >= FX_H) {
        b->y = FX_H - FX_ONE;
        b->vy = -b->vy;
    }
}

/* ================= MAIN ================= */

int main(){
    init_constants();
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* w=SDL_CreateWindow("Boids FAST",SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,WIDTH,HEIGHT,0);
    SDL_Renderer* r=SDL_CreateRenderer(w,-1,SDL_RENDERER_ACCELERATED);
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 frame_start = SDL_GetPerformanceCounter();

    for(int i=0;i<N_BOIDS;i++){
        boids[i].x=fx_from_double(rand()%WIDTH);
        boids[i].y=fx_from_double(rand()%HEIGHT);
        boids[i].vx=fx_from_double(((rand()%100)/100.0)-0.5);
        boids[i].vy=fx_from_double(((rand()%100)/100.0)-0.5);
    }

    int run=1;
    while(run){
        SDL_Event e;
        while(SDL_PollEvent(&e))
            if(e.type==SDL_QUIT) run=0;

        SDL_SetRenderDrawColor(r,0,0,0,255);
        SDL_RenderClear(r);
        SDL_SetRenderDrawColor(r,255,255,255,255);

        for(int i=0;i<N_BOIDS;i++){
            reynolds(i);
            update_boid(i);
            SDL_Rect p={boids[i].x>>FX_SHIFT,boids[i].y>>FX_SHIFT,3,3};
            SDL_RenderFillRect(r,&p);
        }
        SDL_RenderPresent(r);

        Uint64 end = SDL_GetPerformanceCounter();
                double delta = (double)(end - frame_start) / (double)freq;


        const double target = 1.0 / 60.0;
        if (delta < target) {
            Uint32 wait_ms = (Uint32)((target - delta) * 1000.0);
            SDL_Delay(wait_ms);
            end = SDL_GetPerformanceCounter();
            delta = (double)(end - frame_start) / (double)freq;
        }

        double fps = 1.0 / (delta > 0 ? delta : 1.0);
        printf("FPS: %.2f\n", fps);
        frame_start = end;
        
    }
    SDL_Quit();
}
