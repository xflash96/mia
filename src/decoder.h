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

    void decode(uint8_t *buf, int length, int (*on_frame)(AVFrame *frame));
    int nframe;

    uint8_t *buf;
    int buf_length;
    int buf_offset;
    uint8_t *inbuf;
    AVFormatContext *ic;
    AVCodecContext *ctx;
    AVPacket pkt;
    AVFrame *frame;
};
