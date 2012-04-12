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
static void pgm_save(unsigned char *src, int wrap, int width, int height,
                     char *filename)
{
    FILE *f;
    int i, j;

    fprintf(stderr, "wrap = %d\n", wrap);
    f=fopen(filename,"w");
    fprintf(f,"P6\n%d %d %d\n",width,height,255);

  const unsigned char *ysrc = src;
  const unsigned char *usrc, *vsrc;
  int yvu = 0;
  unsigned char dest_zero[10000], *dest;

  if (yvu) {
      vsrc = src + width * height;
      usrc = vsrc + (width * height) / 4;
  } else {
      usrc = src + width * height;
      vsrc = usrc + (width * height) / 4;
  }

  for (i = 0; i < height; i++) {
      dest = dest_zero;
      for (j = 0; j < width; j += 2) {
#if 1 /* fast slightly less accurate multiplication free code */
          int u1 = (((*usrc - 128) << 7) +  (*usrc - 128)) >> 6;
          int rg = (((*usrc - 128) << 1) +  (*usrc - 128) +
                  ((*vsrc - 128) << 2) + ((*vsrc - 128) << 1)) >> 3;
          int v1 = (((*vsrc - 128) << 1) +  (*vsrc - 128)) >> 1;

          *dest++ = CLIP(*ysrc + v1);
          *dest++ = CLIP(*ysrc - rg);
          *dest++ = CLIP(*ysrc + u1);
          ysrc++;

          *dest++ = CLIP(*ysrc + v1);
          *dest++ = CLIP(*ysrc - rg);
          *dest++ = CLIP(*ysrc + u1);
#endif
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

static void video_decode_example(const char *outfilename, const char *filename)
{
    AVCodec *codec;
    AVCodecContext *c= NULL;
    int frame, got_picture, len;
    FILE *f;
    AVFrame *picture;
    uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    char buf[20];
    AVPacket avpkt;

    av_init_packet(&avpkt);

    /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
    memset(inbuf + INBUF_SIZE, 0, FF_INPUT_BUFFER_PADDING_SIZE);

    printf("Video decoding\n");

    codec = avcodec_find_decoder(CODEC_ID_H264);
    if (!codec) {
        fprintf(stderr, "codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    picture= avcodec_alloc_frame();

    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */
    c->workaround_bugs = 1;
    c->lowres = 0;
    c->idct_algo = FF_IDCT_AUTO;
    c->skip_frame = AVDISCARD_DEFAULT;
    c->skip_idct = AVDISCARD_DEFAULT;
    c->skip_loop_filter = AVDISCARD_DEFAULT;
    c->error_concealment = 3;
    c->pix_fmt = PIX_FMT_YUVJ420P;

#if 0
    fprintf(stderr, "ptr %p\n", codec->pix_fmts);
    int i;
    for(i=0; codec->pix_fmts[i] != -1; i++)
        fprintf(stderr, "%d\t", codec->pix_fmts[i]);
    fputc('\n', stderr);
#endif

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "could not open codec\n");
        exit(1);
    }
    fprintf(stderr, "PIX_FMT = %d\n", c->pix_fmt);

    /* the codec gives us the frame size, in samples */

    f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "could not open %s\n", filename);
        exit(1);
    }

    frame = 0;
    for(;;) {
        avpkt.size = fread(inbuf, 1, INBUF_SIZE, f);
        fprintf(stderr, "size = %d\n", avpkt.size);
        if (avpkt.size == 0)
            break;

        /* NOTE1: some codecs are stream based (mpegvideo, mpegaudio)
           and this is the only method to use them because you cannot
           know the compressed data size before analysing it.

           BUT some other codecs (msmpeg4, mpeg4) are inherently frame
           based, so you must call them with all the data for one
           frame exactly. You must also initialize 'width' and
           'height' before initializing them. */

        /* NOTE2: some codecs allow the raw parameters (frame size,
           sample rate) to be changed at any frame. We handle this, so
           you should also take care of it */

        /* here, we use a stream based decoder (mpeg1video), so we
           feed decoder and see if it could decode a frame */
        avpkt.data = inbuf;
        while (avpkt.size > 0) {
            len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
            if (len < 0) {
                fprintf(stderr, "Error while decoding frame %d\n", frame);
                exit(1);
            }
            fprintf(stderr ,"got frame =%d\n", got_picture);
            if (got_picture) {
                printf("saving frame %3d\n", frame);
                fflush(stdout);

                /* the picture is allocated by the decoder. no need to
                   free it */
                snprintf(buf, sizeof(buf), outfilename, frame);
                pgm_save(picture->data[0], picture->linesize[0],
                         c->width, c->height, buf);
                frame++;
            }
            avpkt.size -= len;
            avpkt.data += len;
        }
    }
    /* some codecs, such as MPEG, transmit the I and P frame with a
       latency of one frame. You must do the following to have a
       chance to get the last frame of the video */
    avpkt.data = NULL;
    avpkt.size = 0;
    len = avcodec_decode_video2(c, picture, &got_picture, &avpkt);
    if (got_picture) {
        printf("saving last frame %3d\n", frame);
        fflush(stdout);

        /* the picture is allocated by the decoder. no need to
           free it */
        snprintf(buf, sizeof(buf), outfilename, frame);
        pgm_save(picture->data[0], picture->linesize[0],
                 c->width, c->height, buf);
        frame++;
    }
    fclose(f);

    avcodec_close(c);
    av_free(c);
    av_free(picture);
    printf("\n");
}

int main(int argc, char **argv)
{

    /* register all the codecs */
    avcodec_register_all();
    av_register_all();

    av_log_set_level(AV_LOG_VERBOSE);

    //    audio_decode_example("/tmp/test.sw", filename);
    video_decode_example("test%d.ppm", argv[1]);

    return 0;
}
