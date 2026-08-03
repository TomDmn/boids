import sdl2
import sdl2.ext

sdl2.ext.init()

window = sdl2.ext.Window("Ma fenêtre SDL2", size=(800, 600))
window.show()

running = True
while running:
    events = sdl2.ext.get_events()
    for event in events:
        if event.type == sdl2.SDL_QUIT:
            running = False

sdl2.ext.quit()
