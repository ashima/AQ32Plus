S=../../src
C=$S/calibration

P=python

CFLAGS=-I $S -I $S/state
CFLAGS+=-O3

OBJS=mcHarness.o mcAccumulate.o magCal.o

all: $C/mcAccumulate.cc mcCollectO.m mcCollectA.m mcAccumulate.m \
     mcHarness

GMA=genMcAccumulate.py
$C/mcAccumulate.cc: $(GMA)
	$P $(GMA) -l cc > $@

mcCollectO.m: $(GMA)
	$P $(GMA) -l matlab -i 1 > $@

mcCollectA.m: $(GMA)
	$P $(GMA) -l matlab -i 3 > $@

mcAccumulate.m: $(GMA)
	$P $(GMA) -l matlab -i 2 > $@

mcHarness.o: mcHarness.cc
	g++ $(CFLAGS) -c -o $@ $^

mcAccumulate.o: $C/mcAccumulate.cc
	g++ $(CFLAGS) -c -o $@ $^

magCal.o: $C/magCal.cc
	g++ $(CFLAGS) -c -o $@ $^

mcHarness: $(OBJS)
	g++ -o $@ $^

clean:
	rm $(OBJS)

