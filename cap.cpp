/*
 * modify from http://v4l2spec.bytesex.org/spec/a16706.htm
 *
 * */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
//#include <libv4l2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class V4LCapture
{
public:
	V4LCapture(const char *dev_name) { 
		this->dev_name = strdup(dev_name); 
		width = 320, height = 240;
		n_buffers = 4;
		count = 0;
		
		open_device();
		init_device();
		start_capturing();
	};
	~V4LCapture() {
		stop_capturing();
		uninit_device();
		close_device(); 
		free(dev_name);
	};

	char *dev_name;
	int fd;
	struct buffer *buffers;
	unsigned char *rgbbuf;
	int width, height;
	int n_buffers;
	int count;

	// V4L2 internal
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	void init_device();
	void open_device();
	void uninit_device();
	void close_device();

	void read_frame();
	void start_capturing();
	void stop_capturing();
	void init_mmap();

	void write_ppm(struct buffer *pbuf, const char* name);
};

struct buffer{
	void *	start;
	size_t	length;

	size_t bytesused;
	int no;
	struct v4l2_buffer buf;
};

void errno_exit(const char *tmpl, ...)
{
	va_list ap;
	va_start(ap, tmpl);
	vfprintf(stderr, tmpl, ap);
	fprintf(stderr, "\terror %d, %s\n", errno, strerror(errno));
	va_end(ap);
	exit(EXIT_FAILURE);
}

int xioctl(int fd, int request, void *arg)
{
	int r;
	for (int i = 0; i < 20; i++) {
		r = ioctl(fd, request, arg);
		if (!(r == -1 && errno == EINTR))
			break;
	}

	return r;
}
// from libv4l
#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
void v4lconvert_yuyv_to_rgb24(const unsigned char *src, unsigned char *dest,
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

      *dest++ = CLIP(src[0] + v1);
      *dest++ = CLIP(src[0] - rg);
      *dest++ = CLIP(src[0] + u1);

      *dest++ = CLIP(src[2] + v1);
      *dest++ = CLIP(src[2] - rg);
      *dest++ = CLIP(src[2] + u1);
      src += 4;
    }
  }
}
void V4LCapture::write_ppm(struct buffer *pbuf, const char *name)
{
	FILE *fp = fopen(name, "w");
	fprintf(fp, "P6\n%d %d 255\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
	v4lconvert_yuyv_to_rgb24((unsigned char*)pbuf->start, rgbbuf,
			fmt.fmt.pix.width, fmt.fmt.pix.height);
	fwrite(rgbbuf, pbuf->bytesused/4*6, 1, fp);
	fclose(fp);
}
void process_image(void *p)
{
	fputc('.', stdout);
	fflush(stdout);
}
void V4LCapture::open_device()
{
	fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if( -1 == fd ){
		errno_exit("Cannot open %s", dev_name);
	}
}
void V4LCapture::init_device()
{
	CLEAR (cap);

	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		errno_exit("VIDIOC_QUERYCAP");
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		errno_exit("%s is not a V4L2 device", dev_name);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		errno_exit("%s does not support streaming", dev_name);
	}

	CLEAR (cropcap);

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		CLEAR(crop);

		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;

		if( -1 == xioctl(fd, VIDIOC_S_CROP, &crop) ){
			//errno_exit("CROP");
			// ignore error
		}
	}


	CLEAR (fmt);
	
	fmt.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width	= width;
	fmt.fmt.pix.height	= height;
	fmt.fmt.pix.pixelformat	= V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;

	if( -1 == xioctl(fd, VIDIOC_S_FMT, &fmt) ){
		errno_exit("VIDIOC_S_FMT");
	}

	rgbbuf = (unsigned char*)calloc(fmt.fmt.pix.sizeimage/4*6, 1);
	if (NULL==rgbbuf){
		errno_exit("calloc");
	}

	init_mmap();
}

void V4LCapture::init_mmap()
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count 	= n_buffers;
	req.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory	= V4L2_MEMORY_MMAP;

	if( -1 == xioctl(fd, VIDIOC_REQBUFS, &req) ){
		errno_exit("VIDIOC_REQBUFS");
	}
	if( req.count != (unsigned)n_buffers ){
		errno_exit("Not enough buffers");
	}
	buffers = (struct buffer*) calloc(req.count, sizeof(*buffers));
	for( int i=0; i<n_buffers; i++ ){
		struct v4l2_buffer buf;

		CLEAR( buf );

		buf.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory	= V4L2_MEMORY_MMAP;
		buf.index	= i;

		if( -1 == xioctl(fd, VIDIOC_QUERYBUF, &buf) ){
			errno_exit("VIDIOC_QUERYBUF");
		}

		buffers[i].length = buf.length;
		buffers[i].start =
			mmap(NULL,
				buf.length,
				PROT_READ | PROT_WRITE,
				MAP_SHARED,
				fd,
				buf.m.offset);
		if( MAP_FAILED == buffers[i].start ){
			errno_exit("Fail to mmap %d-th buffer of %d",
					i, n_buffers);
		}
	}
}
void V4LCapture::start_capturing()
{
	for( int i=0; i<n_buffers; i++){
		struct v4l2_buffer buf;

		CLEAR( buf );

		buf.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory	= V4L2_MEMORY_MMAP;
		buf.index	= i;

		if( -1 == xioctl(fd, VIDIOC_QBUF, &buf) ){
			errno_exit("VIDIOC_QBUF");
		}
	}

	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if( -1 == xioctl(fd, VIDIOC_STREAMON, &type) ){
		errno_exit("VIDIOC_STREAMON");
	}
}
void V4LCapture::read_frame()
{
	struct v4l2_buffer buf;

	CLEAR( buf );

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if( -1 == xioctl(fd, VIDIOC_DQBUF, &buf) ){
		errno_exit("VIDIOC_DQBUF");
	}
	
	struct buffer* pbuf = &buffers[buf.index];

	pbuf->bytesused = buf.bytesused;
	pbuf->no = count++;

	char out_name[20];
	sprintf(out_name, "out%d.ppm", pbuf->no);
	write_ppm(pbuf, out_name);
	//process_image(buffers[buf.index].start);

	if( -1 == xioctl(fd, VIDIOC_QBUF, &buf) ){
		errno_exit("VIDIOC_QBUF");
	}
}
void V4LCapture::stop_capturing()
{
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if( -1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
		errno_exit("VIDIOC_STREAMOFF");
	}
}
void V4LCapture::uninit_device()
{
	for( int i=0; i<n_buffers; i++){
		if( -1 == munmap(buffers[i].start, buffers[i].length)){
			errno_exit("munmap");
		}
	}
	free(buffers);
	free(rgbbuf);
}
void V4LCapture::close_device()
{
	if( -1 == close(fd) ){
		errno_exit("close %d", fd);
	}
	fd = -1;
}

int main()
{
	V4LCapture cap("/dev/video0");
	fd_set fds;
	struct timeval tv;
	int r;

	FD_ZERO(&fds);

	FD_SET(cap.fd, &fds);
	tv.tv_sec = 2;
	tv.tv_usec = 0;

	for (int i=0; i<20; i++){
		r = select(cap.fd+1, &fds, NULL, NULL, &tv);
		if (r==-1) {
			errno_exit("select overtime");
		}
		cap.read_frame();
		fputc('.', stderr);
	}
	
	return 0;
}
