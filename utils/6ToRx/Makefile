
CFLAGS=-O3 -Wall
INCS=-I/opt/local/include
LIBDIR=-L/opt/local/lib
LIBS=-lusb-1.0

all:  6toRx

6toRx: 6toRx.c
	gcc $(LIBDIR) $(INCS) $(CFLAGS) -o $@ $^ $(LIBS)


