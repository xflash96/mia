#include <malloc.h>
#include <assert.h>
#include "decoder.h"

extern "C" {
int read_from_stream(void *opaque, uint8_t *buf, int buf_size)
{
    FFStreamDecoder *d = (FFStreamDecoder *)opaque;
    int len;
    int remains = d->buf_length - d->buf_offset;

    if (buf_size > remains)
        len = remains;
    else
        len = buf_size;

    fprintf(stderr, "buf: length = %d, offset = %d ", d->buf_length, d->buf_offset);
    fprintf(stderr, "req = %d, memcpy len = %d\n", buf_size, len);
    memcpy(buf, d->buf+d->buf_offset, len);
    d->buf_offset += len;

    return len;
}
}

FFStreamDecoder::FFStreamDecoder(const char *codec_name)
{
    ic = NULL;
    iformat = NULL;
    options = NULL;
    pb = NULL;
    ctx = NULL;
    buf = NULL;
    buf_offset = 0;
    buf_length = 0;

    iformat = av_find_input_format(codec_name);
    codec = avcodec_find_decoder_by_name(codec_name);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    inbuf = (uint8_t *)malloc(INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE);
    assert(inbuf);
    memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);
}

void FFStreamDecoder::setup(){
    // set input callback
    pb = avio_alloc_context(inbuf, INBUF_SIZE, 0, this, read_from_stream, NULL, NULL);
    pb->seekable = 0;
    pb->write_flag = 0;
    ic = avformat_alloc_context();
    ic->pb = pb;
    //ic->flags |= AVFMT_NOFILE;
    //ic->video_codec_id = CODEC_ID_RAWVIDEO;
    if (!av_dict_get(options, "threads", NULL, 0))
        av_dict_set(&options, "threads", "auto", 0);

    // open stream
    int ret = avformat_open_input(&ic, "", iformat, &options);
    //av_find_stream_info(ic);
    fprintf(stderr, "****first****\n");
    if (ret<0) {
        fprintf(stderr, "format error\n");
        exit(1);
    }
    ic->streams[0]->discard = AVDISCARD_DEFAULT;
    av_dict_free(&options);
    
    // configure Codec ctx
    ctx = ic->streams[0]->codec;
    ctx->workaround_bugs = 1;
    ctx->lowres = 0;
    ctx->idct_algo = FF_IDCT_AUTO;
    ctx->skip_frame = AVDISCARD_DEFAULT;
    ctx->skip_idct = AVDISCARD_DEFAULT;
    ctx->skip_loop_filter = AVDISCARD_DEFAULT;
    ctx->error_concealment = 3;
    ctx->pix_fmt = PIX_FMT_YUVJ420P;
    ctx->flags = CODEC_FLAG_EMU_EDGE;
    ctx->request_sample_fmt = AV_SAMPLE_FMT_NONE;


    // open codec
    if (avcodec_open2(ctx, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }

    frame = avcodec_alloc_frame();
    av_init_packet(&pkt);
    nframe = 0;

}

FFStreamDecoder::~FFStreamDecoder()
{
    free(inbuf);
    avcodec_close(ctx);
    av_free(ctx);
    av_free(frame);
}

void FFStreamDecoder::decode(uint8_t *buf, int length, int (*on_frame)(AVFrame *frame))
{
	this->buf = buf;
	this->buf_offset = 0;
	this->buf_length = length;

	if(!ctx){
		setup();
	}

	pkt.data = buf;
	pkt.size = length;
	/*
	   int ret = av_read_frame(ic, &pkt);
	   fprintf(stderr, "= %d\n", ret);
	   if (pkt.size==0)
	   return;
	*/

	while (pkt.size > 0) {
		int got_frame = 0;

		int len = avcodec_decode_video2(ctx, frame, &got_frame, &pkt);
		if (len < 0) {
			fprintf(stderr, "Error while decoding frame %d\n", nframe);
			exit(1);
		}

		if (got_frame) {
			fprintf(stderr, "get frame\n");
			on_frame(frame);
			nframe++;
		}
		pkt.size -= len;
		pkt.data += len;
	}
    return;
}
