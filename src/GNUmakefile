FFMPEG_LIBS := libavdevice libavformat libavfilter libavcodec libswscale libavutil
FFMPEG_CFLAGS := $(shell pkg-config --cflags $(FFMPEG_LIBS))
FFMPEG_LDFLAGS := $(shell pkg-config --libs $(FFMPEG_LIBS))
GLIB_CFLAGS := $(shell pkg-config --cflags glib-2.0 gthread-2.0)
GLIB_LDFLAGS := $(shell pkg-config --libs glib-2.0 gthread-2.0)
GTK2_CFLAGS := $(shell pkg-config --cflags gtk+-2.0)
GTK2_LDFLAGS := $(shell pkg-config --libs gtk+-2.0)
GTKGL_CFLAGS := $(shell pkg-config --cflags gtkglext-1.0)
GTKGL_LDFLAGS := $(shell pkg-config --libs gtkglext-1.0)
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv)
OPENCV_LDFLAGS := $(shell pkg-config --libs opencv)
# OpenCV featrue_2d improperly include -lhighgui, which use gtk+2.0
# So we have to use it for the moment if we don'et want to recompile it.

CFLAGS := -g -Wall -O3 \
	$(GLIB_CFLAGS) \
	$(GTK2_CFLAGS) \
	$(GTKGL_CFLAGS) \
	$(OPENCV_CFLAGS) \
	$(FFMPEG_CFLAGS) 
	 #-Wconversion 
LDFLAGS := -O3 \
	$(GLIB_LDFLAGS) \
	$(GTK2_LDFLAGS) \
	$(GTKGL_LDFLAGS) \
	$(OPENCV_LDFLAGS) \
	$(FFMPEG_LDFLAGS) \
	-lm

CXXFLAGS := $(CFLAGS) -D__STDC_CONSTANT_MACROS
CXX := g++

PROGRAMS := mia

mia: mia.o cap.o decoder.o convert.o stereo.o calib.o cam.o gui.o iothread.o async.o
	$(CXX) $^ -o $@ $(LDFLAGS)

test: test.o cam.o slam.o
	$(CXX) $^ -o $@ $(LDFLAGS)

.PHONY: all clean install
all: $(PROGRAMS)
clean:
	rm -f $(PROGRAMS) core.* *.o

install:
	./${PROGRAMS}
