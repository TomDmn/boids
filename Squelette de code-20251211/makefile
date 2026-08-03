CC = gcc
CFLAGS = -Wall -Wextra `sdl2-config --cflags`
LDFLAGS = `sdl2-config --libs` -lSDL2_ttf -lSDL2_gfx

all: boids

boids: boids.o
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o boids
