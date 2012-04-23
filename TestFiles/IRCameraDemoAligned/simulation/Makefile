MACHINE= $(shell uname -s)
HOSTNAME=-D$(shell uname -n | sed 's/-/_/g' | sed 's/\./_/g')

SOCKETS_DIR = ../utils/

LIBS = -L/usr/lib/ -lglut -lgle -lGLU
INCLUDES = -I/usr/local/include/ -I/usr/include/ -I/usr/include/eigen2
COMPILEFLAG=
LDFLAG=-lrt

OPT_COMP = -DEIGEN_NO_DEBUG -O3 -DNDEBUG
FOPENMP = -fopenmp
PROFILER = #-L/usr/lib/ -lprofiler
CC = g++ $(OPT_COMP) $(COMPILEFLAG) $(HOSTNAME) $(FOPENMP) $(PROFILER)
LD = g++ $(OPT_COMP) $(LDFLAG) $(HOSTNAME) $(FOPENMP) $(PROFILER)
DEBUG = -ggdb
NDEBUG = #-DNDEBUG

target: IRCameraDemo

IRCameraDemo: IRCameraDemo.cpp mathUtils.o drawUtils.o imageloader.o Box.o Plane.o socketInterfaceReceiver.o UDPSocket.o Socket.o
	$(LD) $(INCLUDES) $(DEBUG) $(NDEBUG) -o IRCameraDemo IRCameraDemo.cpp mathUtils.o drawUtils.o imageloader.o Box.o Plane.o socketInterfaceReceiver.o UDPSocket.o Socket.o $(LIBS) $(PROFILER)

################################################
#    Object files                              #
################################################
mathUtils.o: mathUtils.cpp mathUtils.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c mathUtils.cpp $(PROFILER)

drawUtils.o: drawUtils.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c drawUtils.cpp $(PROFILER)

imageloader.o: imageloader.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c imageloader.cpp $(PROFILER)

Box.o: Box.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c Box.cpp $(PROFILER)

Plane.o: Plane.cpp
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c Plane.cpp $(PROFILER)

Socket.o : $(SOCKETS_DIR)/Socket.cpp $(SOCKETS_DIR)/Socket.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(SOCKETS_DIR)/Socket.cpp
	
UDPSocket.o : $(SOCKETS_DIR)/UDPSocket.cpp $(SOCKETS_DIR)/UDPSocket.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c $(SOCKETS_DIR)/UDPSocket.cpp

socketInterfaceReceiver.o: socketInterfaceReceiver.cpp socketInterfaceReceiver.h
	$(CC) $(INCLUDES) $(DEBUG) $(NDEBUG) -c socketInterfaceReceiver.cpp $(PROFILER)

clean:
	rm -r *.o *.gch IRCameraDemo


