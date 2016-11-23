test:main.o recover3d.o
	g++ -o test main.o recover3d.o `pkg-config --cflags opencv` `pkg-config --libs opencv`

recover3d.o:recover3d.cpp recover3d.h Makefile
	g++ -c -o recover3d.o recover3d.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
