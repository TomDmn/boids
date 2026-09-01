#include <SDL.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define WIDTH 1000
#define HEIGHT 600
#define N_BOIDS 1000

#define V_LIM 4
#define R_COHESION 60
#define R_ALIGNEMENT 40
#define R_SEPARATION 8

#define TAILLE_HASH R_COHESION
#define NB_CASE_X (WIDTH / TAILLE_HASH)
#define NB_CASE_Y (HEIGHT / TAILLE_HASH)

typedef struct {
    double x,y;
    double vx,vy;
    double ax, ay;
    
}predator; 

typedef struct {
    int id;
    double x, y;
    double vx, vy;
    double ax, ay;
    int case_x, case_y;
} Boid;

typedef struct hashed{
    int id;
    struct hashed* next;
}hashed;

hashed* grille[NB_CASE_X][NB_CASE_Y] = {0};


Boid boids[N_BOIDS];



void ajouter_boid_à_case(int cx, int cy, int id){
    hashed* c =malloc(sizeof(hashed));
    c->id=id;
    c->next=grille[cx][cy];
    grille[cx][cy]=c;
}

void supprimer_boid_de_case(int cx, int cy, int id){
    hashed** cur= &grille[cx][cy];
    while (*cur){
        if ((*cur)->id==id){
            hashed *victim = *cur;
            *cur=victim->next;
            free(victim);
            break;
        }
        cur = &(*cur)->next;
    }
}

void deplacement_indirect_vers_un_point(Boid* b, double px, double py,double C,double* axs,double* ays){
    double dx=px-b->x;
    double dy=py-b->y;
    double n = sqrt(dx*dx + dy*dy);
    if (n==0){
        return;
    }

    double ux=dx/n;
    double uy=dy/n;

    *axs=C*ux;
    *ays=C*uy;
}



void update(Boid* b){
    b->vx+=b->ax;
    b->vy+=b->ay;

    double v2= b->vx*b->vx + b->vy*b->vy;
    if (v2 > V_LIM*V_LIM) {
        double n=sqrt(v2);
        b->vx=b->vx*V_LIM/n;
        b->vy=b->vy*V_LIM/n;
    }

    b->x +=b->vx;
    b->y +=b->vy;

    if (b->x<0) b->x += WIDTH;
    if (b->x>= WIDTH) b->x -=WIDTH;
    if (b->y<0) b->y +=HEIGHT;
    if (b->y >=HEIGHT) b->y -=HEIGHT;

    int ncx =b->x/TAILLE_HASH;
    int ncy =b->y/TAILLE_HASH;

    if (ncx !=b->case_x||ncy !=b->case_y) {
        supprimer_boid_de_case(b->case_x, b->case_y, b->id);
        b->case_x =ncx;
        b->case_y =ncy;
        ajouter_boid_à_case(ncx, ncy, b->id);
    }
}

void Reynold(Boid *b, double* axs, double* ays){
    double Gx=0,Gy=0;
    double Sx=0,Sy=0;
    double vx_ali=0,vy_ali=0;
    int nG=1, nA=0, nS=0;
    for (int dx=-1; dx<=1; dx++){
        for (int dy=-1; dy<=1; dy++){
            int case_x= (b->case_x+dx+NB_CASE_X)%NB_CASE_X;
            int case_y= (b->case_y+dy+NB_CASE_Y)%NB_CASE_Y;
            for(hashed* h=grille[case_x][case_y]; h; h=h->next){
                Boid* b1 =&boids[h->id];
                if (b1==b) continue;
                double dx=b1->x-b->x;
                if (dx>WIDTH/2) dx -= WIDTH;
                if (dx<-WIDTH/2) dx += WIDTH;

                double dy=b1->y-b->y;
                if (dy>HEIGHT/2) dy-=HEIGHT;
                if (dy<-HEIGHT/2) dy+=HEIGHT;

                double dist=dx*dx+dy*dy;

                if(dist<=R_COHESION*R_COHESION){
                    Gx+=dx;
                    Gy+=dy;
                    nG+=1;
                    if(dist<=R_ALIGNEMENT*R_ALIGNEMENT){
                        vx_ali+=b1->vx;
                        vy_ali+=b1->vy;
                        nA+=1;
                        if(dist<=R_SEPARATION*R_SEPARATION){
                            Sx-=dx;
                            Sy-=dy;
                            nS+=1;
    
                        }
                    }
                }

            }
        }
    }
    double ax_coh=0, ay_coh=0, ax_sep=0, ay_sep=0, ax_ali=0, ay_ali=0;

    if (nS!=0){
        Sx*=1.0/nS;
        Sy*=1.0/nS;
        deplacement_indirect_vers_un_point(b,Sx+b->x,Sy+b->y,0.02,&ax_sep,&ay_sep);
    }

    Gx*=1.0/nG;
    Gy*=1.0/nG;
    deplacement_indirect_vers_un_point(b,Gx+b->x,Gy+b->y,0.002,&ax_coh,&ay_coh);
    if (nA!=0){
        vx_ali*=1.0/nA;
        vy_ali*=1.0/nA;
        double norme=sqrt(vx_ali*vx_ali+vy_ali*vy_ali);
        double ux=vx_ali/norme;    
        double uy=vy_ali/norme;
        ax_ali=0.004*ux;
        ay_ali=0.004*uy;
    }
    *axs=ax_ali+ax_coh+ax_sep;
    *ays=ay_ali+ay_coh+ay_sep;
}

double dist(Boid a, Boid b){
    double dx = a.x -b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);

}

int main() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* w = SDL_CreateWindow(
        "Boids",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );

    SDL_Renderer* r = SDL_CreateRenderer(w, -1, SDL_RENDERER_ACCELERATED);

    for (int i=0;i<N_BOIDS;i++) {
        boids[i].id = i;
        boids[i].x = rand()%WIDTH;

        boids[i].case_x = boids[i].x / TAILLE_HASH;
        boids[i].case_y = boids[i].y / TAILLE_HASH;
        ajouter_boid_à_case(boids[i].case_x, boids[i].case_y, i);
    }
    Uint64 freq = SDL_GetPerformanceFrequency();
    Uint64 début = SDL_GetPerformanceCounter();
    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e))
            if (e.type == SDL_QUIT) running = 0;

        SDL_SetRenderDrawColor(r,0,0,0,255);
        SDL_RenderClear(r);

        SDL_SetRenderDrawColor(r,255,255,255,255);

        for (int i=0;i<N_BOIDS;i++) {
            Reynold(&boids[i], &boids[i].ax, &boids[i].ay);
            update(&boids[i]);
            int size = 2;
            SDL_Rect carre = {(int)boids[i].x, (int)boids[i].y, size, size};
            SDL_RenderFillRect(r, &carre);
        }

        SDL_RenderPresent(r);
        Uint64 fin = SDL_GetPerformanceCounter();

        double delta = (double)(fin-début)/(double)(freq);

        double fps = 1.0/delta;
        printf("FPS: %.2f \n",fps);

        début=fin;
        //SDL_Delay(8);
    }
   

    SDL_Quit();
    return 0;
}


#define FX_SHIFT 16
#define FX_ONE (1 << FX_SHIFT)
typedef int32_t fx;

fx fx_mul(fx a, fx b) {
    return (fx)((int64_t)a * b >> FX_SHIFT);
}
fx fx_from_double(double d) {
    return (fx)(d * FX_ONE);
}