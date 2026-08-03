import sys
import random
import sdl2
import sdl2.ext


def draw_point(renderer, x, y):
    renderer.draw_point((x, y))

def draw_thick_point(renderer, x, y, size):
    renderer.fill((x, y, size, size))

def main():
    width, height = 600, 800

    sdl2.ext.init()
    size = 5
    x1,y1=300,300
    x2,y2=300,500
    vx1,vy1=0.1,0.1
    vx2,vy2=-0.1,-0.1
    ax1,ay1=0,0
    ax2,ay2=0,0

    pos={1:[x1,y1],2:[x2,y2]}
    v={1:[vx1,vy1],2:[vx2,vy2]}
    a={1:[ax1,ay1],2:[ax2,ay2]}
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
        draw_thick_point(renderer, x1, y1, size)
        draw_thick_point(renderer, x2, y2, size)

        x1 += vx1
        x2 += vx2
        y1 += vy1
        y2 += vy2
        vx1 += ax1
        vx2 += ax2
        vy1 += ay1
        vy2 += ay2

        renderer.present()

    sdl2.ext.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
