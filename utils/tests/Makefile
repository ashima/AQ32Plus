

all: eMatAxisPermTest

eMatAxisPermTest: eMatAxisPermTest.o
	gcc -o $@ eMatAxisPermTest.o

eMatAxisPermTest.o: eMatAxisPermTest.c
	gcc -I ../../src -o $@ -c $^

