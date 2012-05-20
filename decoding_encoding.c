/*
 * Copyright (c) 2001 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file
 * libavcodec API use example.
 *
 * Note that libavcodec only handles codecs (mpeg, mpeg4, etc...),
 * not file formats (avi, vob, mp4, mov, mkv, mxf, flv, mpegts, mpegps, etc...). See library 'libavformat' for the
 * format handling
 */

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"

#define INBUF_SIZE 4096

/*
 * Video decoding example
 */

#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
static void pgm_save(unsigned char **src, int wrap, int width, int height,
                     char *filename)
{
    FILE *f;
    int i, j;

    fprintf(stderr, "wrap = %d\n", wrap);
    f=fopen(filename,"w");
    fprintf(f,"P6\n%d %d %d\n",width,height,255);
    
#if 0
    fwrite(src, height+height/2, width, f);
    fclose(f);
    return;
#endif

  unsigned char *ysrc = src[0];
  const unsigned char *usrc, *vsrc;
  unsigned char dest_zero[10000], *dest;

  vsrc = src[2];// src + width * height;
  usrc = src[1];//vsrc + (width * height) / 4;

  for (i = 0; i < height; i++) {
      dest = dest_zero;
      for (j = 0; j < width; j += 2) {
          int u1 = (((*usrc - 128) << 7) +  (*usrc - 128)) >> 6;
          int rg = (((*usrc - 128) << 1) +  (*usrc - 128) +
                  ((*vsrc - 128) << 2) + ((*vsrc - 128) << 1)) >> 3;
          int v1 = (((*vsrc - 128) << 1) +  (*vsrc - 128)) >> 1;

          if(*ysrc <16)
              *ysrc = 16;
          else if(*ysrc>235)
              *ysrc = 235;
          *dest++ = CLIP(*ysrc + v1);
          *dest++ = CLIP(*ysrc - rg);
          *dest++ = CLIP(*ysrc + u1);
          ysrc++;

          if(*ysrc <16)
              *ysrc = 16;
          else if(*ysrc>235)
              *ysrc = 235;
          *dest++ = CLIP(*ysrc + v1);
          *dest++ = CLIP(*ysrc - rg);
          *dest++ = CLIP(*ysrc + u1);

          ysrc++;
          usrc++;
          vsrc++;
      }
      /* Rewind u and v for next line */
      if (!(i&1)) {
          usrc -= width / 2;
          vsrc -= width / 2;
      }
      fwrite(dest_zero, dest-dest_zero, 1, f);
  }

    fclose(f);
}

static int read_from_stream(void *opaque, uint8_t *buf, int buf_size)
{
    FILE *f = (FILE*)opaque;
    int n = fread(buf, 1, buf_size, f);
    return n;
}


static void video_decode_example(const char *outfilename, const char *filename)
{
    AVFormatContext *ic = NULL;
    AVInputFormat *iformat = NULL;
    AVIOContext *pb = NULL;
    AVCodec *codec;
    AVCodecContext *ctx= NULL;
    int nframe, got_frame, len;
    FILE *f;
    AVFrame *frame;
    uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    char buf[20];
    AVPacket pkt;

    av_init_packet(&pkt);

    memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);

    f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "could not open %s\n", filename);
        exit(1);
    }

    iformat = av_find_input_format("h264");
    pb = avio_alloc_context(inbuf, INBUF_SIZE, 0, f, read_from_stream, NULL, NULL);
    pb->seekable = 0;
    pb->write_flag = 0;
    ic = avformat_alloc_context();
    ic->pb = pb;
    int ret = avformat_open_input(&ic, "", iformat, NULL);
    if (ret<0) {
        fprintf(stderr, "format error\n");
        exit(1);
    }
    
    ctx = ic->streams[0]->codec;
    codec = avcodec_find_decoder(CODEC_ID_H264);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    //ctx = avcodec_alloc_context3(codec);
    frame = avcodec_alloc_frame();

    ctx->workaround_bugs = 1;
    ctx->lowres = 0;
    ctx->idct_algo = FF_IDCT_AUTO;
    ctx->skip_frame = AVDISCARD_DEFAULT;
    ctx->skip_idct = AVDISCARD_DEFAULT;
    ctx->skip_loop_filter = AVDISCARD_DEFAULT;
    ctx->error_concealment = 3;
    ctx->pix_fmt = PIX_FMT_YUVJ420P;

    ctx->flags = CODEC_FLAG_EMU_EDGE | CODEC_FLAG_MV0;
    ctx->request_sample_fmt = AV_SAMPLE_FMT_NONE;
    /*
    ctx->delay = 2;
    ctx->width = 1920;
    ctx->height = 1080;
    */

    if (avcodec_open2(ctx, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }
    /* open it */
    fprintf(stderr, "PIX_FMT = %d\n", ctx->pix_fmt);
    ic->streams[0]->discard = AVDISCARD_DEFAULT;

    /* the codec gives us the frame size, in samples */

    nframe = 0;
    for(;;) {
        av_init_packet(&pkt);
        /*
        pkt.size = fread(inbuf, 1, INBUF_SIZE, f);
        pkt.data = inbuf;
        fprintf(stderr, "size = %d\n", pkt.size);
        */
        pkt.data = NULL;
        pkt.size = 0;
        pkt.stream_index = 0;
        av_read_frame(ic, &pkt);
        if (pkt.size == 0)
            break;

        while (pkt.size > 0) {
            got_frame = 0;
            len = avcodec_decode_video2(ctx, frame, &got_frame, &pkt);
            if (len < 0) {
                fprintf(stderr, "Error while decoding frame %d\n", nframe);
                exit(1);
            }
            fprintf(stderr ,"got frame =%d\n", got_frame);
            if (got_frame) {
                printf("saving frame %3d\n", nframe);
                fflush(stdout);

                snprintf(buf, sizeof(buf), outfilename, nframe);
                pgm_save(frame->data, frame->linesize[0],
                         ctx->width, ctx->height, buf);
                nframe++;
            }
            pkt.size -= len;
            pkt.data += len;
        }
    }
    /* some codecs, such as MPEG, transmit the I and P frame with a
       latency of one frame. You must do the following to have a
       chance to get the last frame of the video */
    pkt.data = NULL;
    pkt.size = 0;
    len = avcodec_decode_video2(ctx, frame, &got_frame, &pkt);
    if (got_frame) {
        printf("saving last frame %3d\n", nframe);
        fflush(stdout);

        /* the frame is allocated by the decoder. no need to
           free it */
        snprintf(buf, sizeof(buf), outfilename, frame);
        pgm_save(frame->data, frame->linesize[0],
                 ctx->width, ctx->height, buf);
        frame++;
    }
    fclose(f);

    avcodec_close(ctx);
    av_free(ctx);
    av_free(frame);
    printf("\n");
}

int main(int argc, char **argv)
{

    /* register all the codecs */
    avcodec_register_all();
    av_register_all();

    av_log_set_level(AV_LOG_DEBUG);

    //    audio_decode_example("/tmp/test.sw", filename);
    video_decode_example("test%d.pgm", "data.h264");

    return 0;
}
