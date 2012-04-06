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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class V4LCapture
{
public:
	V4LCapture(const char *dev_name) { 
		this->dev_name = strdup(dev_name); 
		width = 320, height = 240;
		n_buffers = 4;
		
		open_device();
		init_device();
		start_capturing();
	};
	~V4LCapture() {
		stop_capturing();
		uninit_device();
		close_device(); 
	};

	char *dev_name;
	int fd;
	struct buffer *buffers;
	int width, height;
	int n_buffers;

	void init_device();
	void open_device();
	void uninit_device();
	void close_device();

	void read_frame();
	void start_capturing();
	void stop_capturing();
	void init_mmap();
};

struct buffer{
	void *	start;
	size_t	length;
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
	do r = ioctl(fd, request, arg);
	while ( -1 == r && EINTR == errno );

	return r;
}
void process_image(void *p)
{
	fputc('.', stdout);
	fflush(stdout);
}
void V4LCapture::open_device()
{
	fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if( fd == -1 ){
		errno_exit("Cannot open %s", dev_name);
	}
}
void V4LCapture::init_device()
{
	// struct v4l2_capability cap;
	//struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	CLEAR(crop);

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop.c.left = 0;
	crop.c.top = 0;
	crop.c.width = width*24;
	crop.c.height = height*24;

	if( -1 == xioctl(fd, VIDIOC_S_CROP, &crop) ){
		//errno_exit("CROP");
		// ignore error
	}

	CLEAR(fmt);
	
	fmt.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width	= width;
	fmt.fmt.pix.height	= height;
	fmt.fmt.pix.pixelformat	= V4L2_PIX_FMT_RGB24;
	fmt.fmt.pix.field	= V4L2_FIELD_ANY;

	if( -1 == xioctl(fd, VIDIOC_S_FMT, &fmt) ){
		errno_exit("fmt");
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

	process_image(buffers[buf.index].start);

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
	return 0;
}
