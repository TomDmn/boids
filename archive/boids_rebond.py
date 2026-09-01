import sys
import random
import sdl2
import sdl2.ext
import random

v_lim=0.8


def draw_thick_point(renderer, x, y, size):
    renderer.fill((x, y, size, size))


class Boid:
    def __init__(self, x, y, vx, vy,case_x,case_y, ax=0, ay=0):
        self.x= x
        self.y= y
        self.vx= vx
        self.vy= vy
        self.case_x=case_x
        self.case_y=case_y
        self.ax= ax
        self.ay= ay

    def update(self):
        self.vx += self.ax
        self.vy+= self.ay
        v2=self.vx**2+self.vy**2
        if v2>v_lim**2:
            norm= (v2)**0.5
            self.vx= self.vx *v_lim/norm
            self.vy= self.vy *v_lim/norm

        self.x += self.vx
        self.y += self.vy


def deplacement_indirect_vers_un_point(self, point_fixe, C_coh):
    px,py=point_fixe
    dx=px-self.x
    dy=py-self.y
    norme=(dx*dx + dy*dy)**0.5

    if norme == 0:
        return (0,0)  

    ux=dx/norme
    uy=dy/norme

    return(ux*C_coh,uy*C_coh)
    

    

def main():
    width, height = 400, 400
    rayon_cohesion=100
    taille_hash=rayon_cohesion
    nb_case_x=width//taille_hash
    nb_case_y=height//taille_hash 
    rayon_alignement=70
    rayon_separation=15

    def Reynolds(b):
        Gx,Gy=0,0
        Sx,Sy=0,0
        nG=0
        vx_ali,vy_ali=0,0
        nA=0
        nS=0
        
        

        for key, b1 in boids.items():
            dx=(b1.x-b.x)
            dy=(b1.y-b.y)
            dist=dx*dx+dy*dy
                
            if dist<=rayon_cohesion**2:
                Gx+=dx
                Gy+=dy
                nG+=1
                if dist<=rayon_alignement**2 and b1!=b:
                    vx_ali+= b1.vx
                    vy_ali+=b1.vy
                    nA+=1
                    if dist<=rayon_separation**2:
                        Sx-=dx
                        Sy-=dy
                        nS+=1

        axs,ays=0,0

        if nS!=0:
            Sx*=1/nS
            Sy*=1/nS
            axs,ays=deplacement_indirect_vers_un_point(b,(Sx+b.x,Sy+b.y),0.02)
                    

        Gx*=1/nG
        Gy*=1/nG
        axc,ayc=deplacement_indirect_vers_un_point(b,(Gx+b.x,Gy+b.y),0.008)
        ax_ali,ay_ali=0,0
        if nA!=0:
            vx_ali*=1/nA
            vy_ali*=1/nA
            norm=(vx_ali**2+vy_ali**2)**(1/2)
            ux,uy=vx_ali/norm,vy_ali/norm
            ax_ali,ay_ali=0.012*ux,0.012*uy
        
        return(axc+ax_ali+axs,ayc+ay_ali+ays)

    


    sdl2.ext.init()
    size = 5

    boids={}
    for i in range(200):
        x,y=random.randint(0, width),random.randint(0, height)
        vx,vy=random.uniform(-0.2, 0.2),random.uniform(-0.2, 0.2)
        boids[i]=Boid(x,y,vx,vy,x//taille_hash,y//taille_hash)



    # Création fenêtre + renderer haut-niveau
    window = sdl2.ext.Window("Boids Simulation", size=(width, height))
    window.show()

    renderer = sdl2.ext.Renderer(window)
    running = True

    while running:
        # Gestion des événements
        events = sdl2.ext.get_events()
        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False

        # Effacer écran en noir
        renderer.color = sdl2.ext.Color(0, 0, 0)
        renderer.clear()

        # Dessiner point blanc aléatoire
        renderer.color = sdl2.ext.Color(255, 255, 255)

        for key, b in boids.items():
            draw_thick_point(renderer, b.x, b.y, size)
            b.update()
            b.ax,b.ay=Reynolds(b)


            if b.x-size <=0 or b.x+size>=width:
                b.vx*=-1
            if b.y-size <=0 or b.y+size>=height:
                b.vy*=-1
            
        renderer.present()

        
    sdl2.ext.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
