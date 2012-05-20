#include <linux/videodev2.h>
#include <opencv2/core/core_c.h>

void yuyv_to_iplimage()
{
	IplImage frame;
	cvInitImageHeader( &frame,
			   cvSize( fmt.fmt.pix.width,
				   fmt.fmt.pix.height ),
			   IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4 );
	frame.imageData = (char*)cvAlloc(frame.imageSize);
	cvFree(&frame.imageData);
}

void yuv420_to_gtk_image()
{
}

// from libv4l
#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
void v4lconvert_yuyv_to_bgr24(const unsigned char *src, unsigned char *dest,
  int width, int height)
{
  int j;

  while (--height >= 0) {
    for (j = 0; j < width; j += 2) {
      int u = src[1];
      int v = src[3];
      int u1 = (((u - 128) << 7) +  (u - 128)) >> 6;
      int rg = (((u - 128) << 1) +  (u - 128) +
		((v - 128) << 2) + ((v - 128) << 1)) >> 3;
      int v1 = (((v - 128) << 1) +  (v - 128)) >> 1;

      *dest++ = CLIP(src[0] + u1);
      *dest++ = CLIP(src[0] - rg);
      *dest++ = CLIP(src[0] + v1);

      *dest++ = CLIP(src[2] + u1);
      *dest++ = CLIP(src[2] - rg);
      *dest++ = CLIP(src[2] + v1);
      src += 4;
    }
  }
}

int64_t timespec_to_ms(struct timespec *t)
{
	return t->tv_sec*1000 + t->tv_nsec/1000000;
}

static void v4lconvert_yuv420_to_bgr24(uint8_t *ysrc, uint8_t *usrc, uint8_t *vsrc, uint8_t *dst, int width, int height)
{
    int i, j;

    unsigned char dest_zero[10000], *dest;

    for (i = 0; i < height; i++) {
        dest = dest_zero;
        for (j = 0; j < width; j += 2) {
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

            ysrc++;
            usrc++;
            vsrc++;
        }
        /* Rewind u and v for next line */
        if (!(i&1)) {
            usrc -= width / 2;
            vsrc -= width / 2;
        }
    }
}

#if 0
void write_ppm(const char *name)
{
	FILE *fp = fopen(name, "w");
	fprintf(fp, "P6\n%d %d 255\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
	fwrite(frame.imageData, frame.imageSize, 1, fp);
	fclose(fp);
}

int main()
{
	V4LCapture cap("/dev/video0");
	fd_set fds;
	int r;
	struct timespec t0, t1;

	r = clock_gettime(CLOCK_MONOTONIC, &t0);
	assert(r == 0);

	for (int i=0; i<300; i++) {
		FD_ZERO(&fds);
		FD_SET(cap.fd, &fds);
		r = select(cap.fd + 1, &fds, NULL, NULL, NULL);
		if (r==-1) {
			errno_exit("select overtime");
		}

		if (FD_ISSET(cap.fd, &fds))
			cap.read_frame();
	}
	
	r = clock_gettime(CLOCK_MONOTONIC, &t1);
	assert(r == 0);

	printf("%lld\n", (long long)(timespec_to_ms(&t1) - timespec_to_ms(&t0)));

	return 0;
}
#endif
