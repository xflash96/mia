CFLAGS := -g -Wall -Wconversion -O3 $(shell pkg-config opencv --cflags)
CXXFLAGS := $(CFLAGS)
LDFLAGS := -lm -lv4l2 $(shell pkg-config opencv --libs)

PROGRAMS := cap

.PHONY: all clean
all: $(PROGRAMS)
clean:
	rm -f $(PROGRAMS) core.*
