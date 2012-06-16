#ifndef __DECODER_H__
#define __DECODER_H__

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

#define INBUF_SIZE 4096

class FFStreamDecoder
{
public:
    FFStreamDecoder(const char *codec_name);
    ~FFStreamDecoder();

    void read(uint8_t *buf, int length);
    AVFrame *decode();

    void setup();

    uint8_t *buf;
    int buf_length;
    int buf_offset;
    uint8_t *inbuf;
    AVFormatContext *ic;
    AVCodecContext *ctx;
    AVIOContext *pb;
    AVCodec *codec;
    AVPacket pkt;
    AVFrame *frame;
    AVInputFormat *iformat;
    AVDictionary *options;
};

#endif
