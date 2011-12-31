#   Makefile - antix project
#   version 3
#   Richard Vaughan  

# this should work on Linux with MESA
#GLUTLIBS = -L/usr/X11R6/lib -lGLU -lGL -lglut -lX11 -lXext -lXmu -lXi
#GLUTFLAGS = -I/usr/local/include/GL

# this works on Mac OS X
GLUTFLAGS = -framework OpenGL -framework GLUT

CC = g++
CXXFLAGS = -g -O3 -Wall $(GLUTFLAGS)
#CXXFLAGS = -g -Wall $(GLUTFLAGS)
LIBS =  -g -lm $(GLUTLIBS)

HDR = tri.h controller.h
SRC = tri.cc controller.cc gui.cc main.cc

all: tri

tri: $(SRC) $(HDR)
	$(CC) $(CXXFLAGS) $(LIBS) -o $@ $(SRC) 

clean:
	rm -f *.o tri

