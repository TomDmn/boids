import sys
import random
import sdl2
import sdl2.ext
import random
import numpy as np

image=0
v_lim=0.8
width, height = 1000, 600
rayon_cohesion=100
taille_hash=rayon_cohesion
rayon_alignement=70
rayon_separation=15
nb_case_x=width//taille_hash
nb_case_y=height//taille_hash 
case={(i,j): [] for i in range (nb_case_x) for j in range (nb_case_y)}


def cases_autour(i,j):
    return [case[((i-1)%nb_case_x,(j-1)%nb_case_y)],case[((i-1)%nb_case_x,j%nb_case_y)],case[((i-1)%nb_case_x,(j+1)%nb_case_y)],case[(i%nb_case_x,(j-1)%nb_case_y)],case[(i%nb_case_x,j%nb_case_y)],case[(i%nb_case_x,(j+1)%nb_case_y)],case[((i+1)%nb_case_x,(j-1)%nb_case_y)],case[((i+1)%nb_case_x,j%nb_case_y)],case[((i+1)%nb_case_x,(j+1)%nb_case_y)]]



def draw_thick_point(renderer, x, y, size):
    renderer.fill((x, y, size, size))


class Boid:
    def __init__(self,id, x, y, vx, vy,case_x,case_y, ax=0, ay=0):
        self.id= id
        self.x= x
        self.y= y
        self.vx= vx
        self.vy= vy
        self.case_x=case_x
        self.case_y=case_y
        self.ax= ax
        self.ay= ay
        case[case_x,case_y].append(id)
    def update(self):
        self.vx += self.ax
        self.vy+= self.ay
        v2=self.vx**2+self.vy**2
        if v2>v_lim**2:
            norm= (v2)**0.5
            self.vx= self.vx *v_lim/norm
            self.vy= self.vy *v_lim/norm
        self.x+= self.vx
        self.y+= self.vy
        if self.x < 0:
            self.x += width
        elif self.x > width:
            self.x -= width
        if self.y < 0:
            self.y += height
        elif self.y > height:
            self.y -= height
        if (self.case_x,self.case_y) != (self.x//taille_hash,self.y//taille_hash):
            case[self.case_x,self.case_y].remove(self.id)
            self.case_x,self.case_y=int(self.x//taille_hash),int(self.y//taille_hash)
            case[self.case_x,self.case_y].append(self.id)


def deplacement_indirect_vers_un_point(self, point_fixe, C):
    px,py=point_fixe
    dx=px-self.x
    dy=py-self.y
    norme=(dx*dx + dy*dy)**0.5

    if norme == 0:
        return (0,0)  

    ux=dx/norme
    uy=dy/norme

    return(ux*C,uy*C)
    

    
def main():
    def Reynolds(b):
        Gx,Gy=0,0
        Sx,Sy=0,0
        nG=0
        vx_ali,vy_ali=0,0
        nA=0
        nS=0
        for case in cases_autour(b.case_x,b.case_y):
            for i in case:
                b1=boids[i]
                dx=min((b1.x-b.x),width-abs(b1.x-b.x))
                dy=min((b1.y-b.y), height-abs(b1.y-b.y))
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
        x,y=random.randint(0, width-1),random.randint(0, height-1)
        vx,vy=random.uniform(-0.2, 0.2),random.uniform(-0.2, 0.2)
        case_x=x//taille_hash
        case_y=y//taille_hash
        boids[i]=Boid(i,x,y,vx,vy,case_x,case_y)

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

        renderer.present()

        

    sdl2.ext.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
