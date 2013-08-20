CC=gcc
CCC=g++

CFLAGS= -O3

# --fast-math -fno-math-errno -fno-trapping-math -fno-signaling-nans \
#	-fsingle-precision-constant

CCFLAGS= -fno-rtti -fno-exceptions -Weffc++


SD=../../src/state
INCDIRS=$(SD)
vpath %.c $(SD)
vpath %.cc $(SD)

CSRCS=
CCSRCS=filter.cc harness.cc \
	assimilate.cc

OBJS=$(CCSRCS:.cc=.o) $(CSRCS:.c=.o)

INCS=$(patsubst %, -I %,$(INCDIRS))

all: state

%.o: %.c
	$(CC) $(CFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -c $^

%.o: %.cc
	$(CCC) $(CFLAGS) $(CCFLAGS) $(DEFS) $(CMSISINC) $(INCS) -o $@ -c $^

state: $(OBJS)
	$(CCC) $(CFLAGS) $(CCFLAGS) $^ -o $@

