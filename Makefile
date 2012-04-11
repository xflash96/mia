CC=g++
CFLAGS=-g -Wall -Wconversion  -O3 $(shell pkg-config opencv --cflags)
CPPFLAGS=$(CFLAGS)
LDFLAGS=-lm -O3 -lv4l2 $(shell pkg-config opencv --libs)

all: cap

v4l2: v4l2.o
v4l2.o: v4l2.c

cap: cap.o
cap.o: cap.cpp

cap_sample: cap_sample.o
cap_sample.o: cap_sample.c

.PHONY: clean
clean:
	rm -f cap *.o
