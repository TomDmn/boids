#include <SDL2/SDL.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define WIDTH 480
#define HEIGHT 480
#define N_BOIDS 1500

#define NB_BOMB 2
#define W_BOMB 2

#define V_LIM 0.5
#define R_COHESION 60
#define R_ALIGNEMENT 6
#define R_SEPARATION 18

#define TAILLE_HASH R_COHESION
#define NB_CASE_X (WIDTH / TAILLE_HASH)
#define NB_CASE_Y (HEIGHT / TAILLE_HASH)

/* ================= FIXED POINT ================= */
#define FX_SHIFT 16
#define FX_ONE (1 << FX_SHIFT)
typedef int32_t fx;

static inline fx fx_from_double(double d){ return (fx)(d * FX_ONE); }
static inline fx fx_mul(fx a, fx b){ return (fx)(((int64_t)a * b) >> FX_SHIFT); }
static inline fx fx_div(fx a, fx b){ return (fx)(((int64_t)a << FX_SHIFT) / b); }
static inline fx fx_norm2(fx x, fx y){ return (fx)(((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT); }
static inline int64_t fx_norm2_64(fx x, fx y){ return (((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT); }
static inline fx fx_sqrt(fx a){
    double d = (double)a / (double)FX_ONE;
    return fx_from_double(sqrt(d));
}

/* ================= STRUCTURES ================= */
typedef struct {
    int id;
    fx x, y;
    fx vx, vy;
    fx ax, ay;
    fx ax_bomb, ay_bomb;
    int case_x, case_y;
} Boid;

typedef struct Boid_in_Bomb{
    int id_boid;
    struct Boid_in_Bomb* next;
}Boid_in_Bomb;

typedef struct Bomb{
    int id;
    fx x, y;
    int tick;
    int tick_end;
    int size;
    int64_t radius2;
    Boid_in_Bomb* boids;
    int explosion_timer;
    fx blast_x, blast_y;
}Bomb;


typedef struct hashed{
    int id;
    struct hashed* next;
} hashed;

/* ================= GLOBALS ================= */
Boid boids[N_BOIDS];
hashed* grid[NB_CASE_X][NB_CASE_Y] = {0};
Bomb bombs[NB_BOMB] = {0};

fx FX_R_COH2, FX_R_ALIGN2, FX_R_SEP2;
fx FX_W_COH, FX_W_ALIGN, FX_W_SEP;
fx FX_W_BOMB;
fx FX_V_LIM2;
fx FX_W, FX_H;

/* ================= GRID ================= */
static inline void grid_add(int cx, int cy, int id){
    hashed* h = (hashed*)malloc(sizeof(hashed));
    h->id = id;
    h->next = grid[cx][cy];
    grid[cx][cy] = h;
}

static inline void grid_remove(int cx, int cy, int id){
    hashed** cur = &grid[cx][cy];
    while (*cur){
        if ((*cur)->id == id){
            hashed* victim = *cur;
            *cur = victim->next;
            free(victim);
            return;
        }
        cur = &(*cur)->next;
    }
    return;
}
static inline void InBomb_add(int id_bomb, int id_boid){
    Boid_in_Bomb* h = (Boid_in_Bomb*)malloc(sizeof(Boid_in_Bomb));
    h->id_boid = id_boid;
    h->next = bombs[id_bomb].boids;
    bombs[id_bomb].boids = h;
}

static inline void InBomb_remove(int id_bomb, int id_boid){
    Boid_in_Bomb** cur = &bombs[id_bomb].boids;
    while (*cur){
        if ((*cur)->id_boid == id_boid){
            Boid_in_Bomb* victim = *cur;
            *cur = victim->next;
            free(victim);
            return;
        }
        cur = &(*cur)->next;
    }
}


static inline int InBomb_contains(int id_bomb, int id_boid){
    for (Boid_in_Bomb* j = bombs[id_bomb].boids; j; j = j->next){
        if (j->id_boid == id_boid)
            return 1;
    }
    return 0;
}


/* ================= INIT ================= */
static void init_constants(){
    fx r_coh = fx_from_double(R_COHESION);
    fx r_ali = fx_from_double(R_ALIGNEMENT);
    fx r_sep = fx_from_double(R_SEPARATION);
    FX_R_COH2 = fx_mul(r_coh, r_coh);
    FX_R_ALIGN2 = fx_mul(r_ali, r_ali);
    FX_R_SEP2 = fx_mul(r_sep, r_sep);

    FX_W_COH = fx_from_double(0.002);
    FX_W_ALIGN = fx_from_double(0.004);
    FX_W_SEP = fx_from_double(0.02);
    FX_W_BOMB = fx_from_double(W_BOMB);

    fx vl = fx_from_double(V_LIM);
    FX_V_LIM2 = fx_mul(vl, vl);
    FX_W = fx_from_double(WIDTH);
    FX_H = fx_from_double(HEIGHT);
}

/* ================= HELPERS ================= */
static inline void deplacement_indirect_vers_un_point(fx dx, fx dy, fx C, fx* axs, fx* ays){
    fx n = fx_sqrt(fx_norm2(dx, dy));
    if (n == 0) return;
    fx ux = fx_div(dx, n);
    fx uy = fx_div(dy, n);
    *axs += fx_mul(C, ux);
    *ays += fx_mul(C, uy);
}

static void Bomb_Detection(){
    for (int i=0; i<NB_BOMB; i++){
        Bomb* bomb= &bombs[i];
        bomb->tick++;
        if (bomb->tick>=bomb->tick_end){
            bomb->explosion_timer = 15; 
            bomb->blast_x = bomb->x;
            bomb->blast_y = bomb->y;
            for(Boid_in_Bomb* j=bombs[i].boids; j; j=j->next){
                Boid* b1 =&boids[j->id_boid];
                fx Bx=b1->x-bomb->x;
                fx By=b1->y-bomb->y;
                fx dx = b1->x - bomb->x;
                fx dy = b1->y - bomb->y;

                fx dist = fx_sqrt(fx_norm2(dx, dy));
                if (dist == 0) continue;  

                fx ux = fx_div(dx, dist);
                fx uy = fx_div(dy, dist);

                b1->vx += fx_mul(FX_W_BOMB, ux);
                b1->vy += fx_mul(FX_W_BOMB, uy);
            }
            bombs[i].tick = 0;
            bombs[i].x = fx_from_double(rand() % WIDTH);
            bombs[i].y = fx_from_double(rand() % HEIGHT);
            bombs[i].tick_end = rand() % 500; 
            fx rad = fx_from_double(75.0); 
            bombs[i].radius2 = ((int64_t)rad * rad) >> FX_SHIFT;
            bombs[i].boids = NULL;
            
        }
        if (bomb->explosion_timer>0) bomb->explosion_timer--;
    }
}


/* ================= REYNOLDS ================= */
static void Reynold(Boid *b, fx* axs, fx* ays){
    fx Gx=0,Gy=0;
    fx Sx=0,Sy=0;
    fx vx_ali=0,vy_ali=0;
    int nG=1, nA=0, nS=0;

    for (int dx=-1; dx<=1; dx++){ 
        for (int dy=-1; dy<=1; dy++){
            int case_x= (b->case_x+dx+NB_CASE_X)%NB_CASE_X;
            int case_y= (b->case_y+dy+NB_CASE_Y)%NB_CASE_Y;
            for(hashed* h=grid[case_x][case_y]; h; h=h->next){
                Boid* b1 =&boids[h->id];
                if (b1==b) continue;
                fx rx=b1->x-b->x; 
                if (rx > FX_W/2) rx -= FX_W;
                if (rx < -FX_W/2) rx += FX_W;

                fx ry=b1->y-b->y;
                if (ry > FX_H/2) ry -= FX_H;
                if (ry < -FX_H/2) ry += FX_H;

                fx dist=fx_norm2(rx,ry);

                if(dist<=FX_R_COH2){
                    Gx+=rx;
                    Gy+=ry;
                    nG+=1;
                    if(dist<=FX_R_ALIGN2){
                        vx_ali+=b1->vx;
                        vy_ali+=b1->vy;
                        nA+=1;
                        if(dist<=FX_R_SEP2){
                            Sx-=rx;
                            Sy-=ry;
                            nS+=1;
                        }
                    }
                }

            }
        }
    }
    fx ax_coh=0, ay_coh=0, ax_sep=0, ay_sep=0, ax_ali=0, ay_ali=0;

    if (nS!=0){
        Sx/=nS;
        Sy/=nS;
        deplacement_indirect_vers_un_point(Sx, Sy, FX_W_SEP, &ax_sep, &ay_sep);
    }

    Gx/=nG;
    Gy/=nG;
    deplacement_indirect_vers_un_point(Gx, Gy, FX_W_COH, &ax_coh, &ay_coh);
    if (nA!=0){
        vx_ali/=nA;
        vy_ali/=nA;
        fx norme=fx_sqrt(fx_norm2(vx_ali,vy_ali));
        if (norme!=0){
            fx ux=fx_div(vx_ali,norme);    
            fx uy=fx_div(vy_ali,norme);
            ax_ali=fx_mul(FX_W_ALIGN,ux);
            ay_ali=fx_mul(FX_W_ALIGN,uy);
        }
    }
    *axs=ax_ali+ax_coh+ax_sep+b->ax_bomb;
    *ays=ay_ali+ay_coh+ay_sep+b->ay_bomb;
    b->ax_bomb=0;
    b->ay_bomb=0;
}

/* ================= UPDATE ================= */
static void update(Boid* b){
    b->vx+=b->ax;
    b->vy+=b->ay;

    fx v2= fx_norm2(b->vx,b->vy);
    if (v2 > FX_V_LIM2) {
        fx scale = fx_div(fx_from_double(V_LIM), fx_sqrt(v2));
        b->vx = fx_mul(b->vx, scale);
        b->vy = fx_mul(b->vy, scale);
    }

    b->x +=b->vx;
    b->y +=b->vy;

    if (b->x<0) b->x += FX_W;
    if (b->x>= FX_W) b->x -=FX_W;
    if (b->y<0) b->y +=FX_H;
    if (b->y >=FX_H) b->y -=FX_H;

    int ncx =(b->x>>FX_SHIFT)/TAILLE_HASH;
    int ncy =(b->y>>FX_SHIFT)/TAILLE_HASH;

    if (ncx !=b->case_x||ncy !=b->case_y) {
        grid_remove(b->case_x, b->case_y, b->id);
        b->case_x =ncx;
        b->case_y =ncy;
        grid_add(ncx, ncy, b->id);
    }

    for (int i=0; i<NB_BOMB; i++){
        Bomb* bomb= &bombs[i];
        fx dx =b->x-bomb->x;
        fx dy =b->y-bomb->y;
        int64_t d2 =fx_norm2_64(dx,dy);
        if (d2 <= bomb->radius2) {
            if (!InBomb_contains(i, b->id)) {
                InBomb_add(i, b->id);
            }
        } 
        else {
                InBomb_remove(i, b->id);
            }

    }
}

/* ================= MAIN ================= */
int main() {

    init_constants();
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* w = SDL_CreateWindow(
        "Boids",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );

    SDL_Renderer* r = SDL_CreateRenderer(w, -1, SDL_RENDERER_ACCELERATED);
    SDL_ShowCursor(SDL_DISABLE);
    for (int i = 0; i < NB_BOMB; i++){
    bombs[i].id = i;
    bombs[i].x = fx_from_double(rand() % WIDTH);
    bombs[i].y = fx_from_double(rand() % HEIGHT);
    bombs[i].tick = 0;
    bombs[i].tick_end = 500; 
    fx rad = fx_from_double(75.0); 
    bombs[i].radius2 = ((int64_t)rad * rad) >> FX_SHIFT;
    bombs[i].boids = NULL;
    bombs[i].blast_x = bombs[i].x;
    bombs[i].blast_y = bombs[i].y;
    }


    for (int i=0;i<N_BOIDS;i++) {
        boids[i].id = i;
        boids[i].x = fx_from_double(rand()%WIDTH);
        boids[i].y = fx_from_double(rand()%HEIGHT);
        boids[i].vx = fx_from_double(((rand()%100)/250.0)-0.2);
        boids[i].vy = fx_from_double(((rand()%100)/250.0)-0.2);
        boids[i].ax = boids[i].ay = 0;
        boids[i].ax_bomb = boids[i].ay_bomb = 0;

        boids[i].case_x = (boids[i].x >> FX_SHIFT) / TAILLE_HASH;
        boids[i].case_y = (boids[i].y >> FX_SHIFT) / TAILLE_HASH;
        grid_add(boids[i].case_x, boids[i].case_y, i);
    }
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 debut = SDL_GetPerformanceCounter();
    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e))
            if (e.type == SDL_QUIT) running = 0;

        SDL_SetRenderDrawColor(r,0,0,0,255);
        SDL_RenderClear(r);

        Bomb_Detection();
        for (int i=0; i<NB_BOMB; i++){
            int bx = (int)((bombs[i].explosion_timer>0 ? bombs[i].blast_x : bombs[i].x) >> FX_SHIFT);
            int by = (int)((bombs[i].explosion_timer>0 ? bombs[i].blast_y : bombs[i].y) >> FX_SHIFT);
            if (bombs[i].explosion_timer>0){
                int rad = (int)sqrt((double)bombs[i].radius2 / (double)FX_ONE);
                SDL_SetRenderDrawColor(r,255,165,0,180);
                SDL_Rect blast = {bx-rad, by-rad, 2*rad, 2*rad};
                SDL_RenderFillRect(r, &blast);
            }
            SDL_SetRenderDrawColor(r,255,0,0,255);
            SDL_Rect bomb_rect = {bx-4, by-4, 8, 8};
            SDL_RenderFillRect(r, &bomb_rect);
        }

        SDL_SetRenderDrawColor(r,255,255,255,255);
        for (int i=0;i<N_BOIDS;i++) { 
            Reynold(&boids[i], &boids[i].ax, &boids[i].ay);
            update(&boids[i]);
            int size = 2;
            SDL_Rect carre = {(int)(boids[i].x>>FX_SHIFT), (int)(boids[i].y>>FX_SHIFT), size, size};
            SDL_RenderFillRect(r, &carre);

        }

        SDL_RenderPresent(r);
        Uint64 fin = SDL_GetPerformanceCounter();

        double delta = (double)(fin-debut)/(double)(freq);

        double fps = 1.0/delta;
        //printf("FPS: %.2f \n",fps);

        debut=fin;
        //SDL_Delay(8);
    }
   

    SDL_Quit();
    return 0;
}
