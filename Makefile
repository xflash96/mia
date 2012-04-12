FFMPEG_LIBS := libavdevice libavformat libavfilter libavcodec libswscale libavutil
FFMPEG_CFLAGS := $(shell pkg-config --cflags $(FFMPEG_LIBS))
FFMPEG_LDFLAGS := $(shell pkg-config --libs $(FFMPEG_LIBS))
CFLAGS := -g -Wall -O3 $(shell pkg-config opencv --cflags) $(FFMPEG_CFLAGS) #-Wconversion 
CXXFLAGS := $(CFLAGS)
LDFLAGS := -lv4l2 $(shell pkg-config opencv --libs) $(FFMPEG_LDFLAGS) -lrt -lm
CXX := g++

PROGRAMS := cap decoding_encoding

.PHONY: all clean
all: $(PROGRAMS)
clean:
	rm -f $(PROGRAMS) core.* *.o
