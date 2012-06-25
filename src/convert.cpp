extern "C"{
#include <libswscale/swscale.h>
#include <libavutil/pixdesc.h>
}
#include <sys/time.h>
#include "convert.h"

void errno_exit(const char *tmpl, ...)
{
	va_list ap;
	va_start(ap, tmpl);
	vfprintf(stderr, tmpl, ap);
	fprintf(stderr, "\n\terror %d: %s\n", errno, strerror(errno));
	va_end(ap);
	exit(EXIT_FAILURE);
}

cv::Mat avframe_to_cvmat(AVFrame *frame)
{
	AVFrame dst;
	cv::Mat m;

	memset(&dst, 0, sizeof(dst));

	int w = frame->width, h = frame->height;
	m = cv::Mat(h, w, CV_8UC3);
	dst.data[0] = (uint8_t *)m.data;
	avpicture_fill( (AVPicture *)&dst, dst.data[0], PIX_FMT_BGR24, w, h);

	struct SwsContext *convert_ctx=NULL;
	enum PixelFormat src_pixfmt = (enum PixelFormat)frame->format;
	enum PixelFormat dst_pixfmt = PIX_FMT_BGR24;
	convert_ctx = sws_getContext(w, h, src_pixfmt, w, h, dst_pixfmt,
			SWS_FAST_BILINEAR, NULL, NULL, NULL);
	sws_scale(convert_ctx, frame->data, frame->linesize, 0, h,
			dst.data, dst.linesize);
	sws_freeContext(convert_ctx);

	return m;
}

cv::Mat raw_to_cvmat(uint8_t *data, int width, int height, enum PixelFormat pixel_format)
{
	AVFrame avframe;
	memset(&avframe, 0, sizeof(avframe));
	avframe.format = pixel_format;
	avpicture_fill( (AVPicture *)&avframe, avframe.data[0], pixel_format,
			width, height);
	avframe.width = width;
	avframe.height = height;
	avframe.data[0] = data;

	return avframe_to_cvmat(&avframe);
}

int64_t timespec_to_ns(struct timespec *t)
{
	return t->tv_sec*1000000000L + t->tv_nsec;
}

int64_t timeval_to_ns(struct timeval *t)
{
	return t->tv_sec*1000000000L + t->tv_usec*1000;
}

int64_t time_now_ns()
{
#ifdef __APPLE__
    struct timeval tv;
    int ret = gettimeofday( &tv, NULL );
    return timeval_to_ns(&tv);
#else
    struct timespec tspec;
    int r = clock_gettime(CLOCK_MONOTONIC, &tspec);
    assert(r==0);
    return timespec_to_ns(&tspec);
#endif
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
