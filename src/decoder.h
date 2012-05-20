#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
//#include <libavutil/mathematics.h>
//#include <libavutil/samplefmt.h>

#define INBUF_SIZE 4096

class FFStreamDecoder
{
public:
    FFStreamDecoder(const char *codec_name);
    ~FFStreamDecoder();

    int decode(uint8_t *buf, int length, int (*on_frame)(AVFrame *frame));

    uint8_t *buf;
    int buf_length;
    int buf_offset;
    int nframe;
    AVFormatContext *ic;
    AVCodecContext *ctx;
    AVPacket pkt;
    AVFrame *frame;
};
