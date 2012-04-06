CC=g++
CFLAGS=-g -Wall -Wconversion  -O3
CPPFLAGS=$(CFLAGS)
LDFLAGS=-lm -O3

all: cap
cap: cap.o

cap.o: cap.cpp

.PHONY: clean
clean:
	rm -f cap *.o
