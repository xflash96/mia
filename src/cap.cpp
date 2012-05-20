/*
 * modify from http://v4l2spec.bytesex.org/spec/a16706.htm
 *
 * */
#include <stdint.h>
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
#include <time.h>

#include <opencv2/core/core_c.h>
#include "cap.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))


extern void errno_exit(const char *tmpl, ...);

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

V4LCapture::V4LCapture(const char *dev_name, struct V4LCaptureParam param)
{
	n_buffers = 4;
	sequence = (__u32)0;
	this->dev_name = strdup(dev_name); 
	this->param = param;

	open_device();
	init_device();
	start_capturing();
};

V4LCapture::~V4LCapture() {
	stop_capturing();
	uninit_device();
	close_device(); 
	free(dev_name);
};

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
	fmt.fmt.pix.width	= param.width;
	fmt.fmt.pix.height	= param.height;
	fmt.fmt.pix.pixelformat	= param.pixelformat;//V4L2_PIX_FMT_H264;
	fmt.fmt.pix.field	= V4L2_FIELD_INTERLACED;

	if( -1 == xioctl(fd, VIDIOC_S_FMT, &fmt) ){
		errno_exit("VIDIOC_S_FMT");
	}

	struct v4l2_streamparm setfps;
	
	CLEAR (setfps);

	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps.parm.capture.timeperframe.numerator = 1;
	setfps.parm.capture.timeperframe.denominator = param.fps;
	xioctl (fd, VIDIOC_S_PARM, &setfps);

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
		buffers[i].start = mmap( NULL,
					 buf.length,
					 PROT_READ | PROT_WRITE,
					 MAP_SHARED,
					 fd,
					 buf.m.offset );
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
void V4LCapture::read_frame(int (*onread)(uint8_t *data, struct v4l2_buffer buf))
{
	struct v4l2_buffer buf;

	CLEAR( buf );

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if( -1 == xioctl(fd, VIDIOC_DQBUF, &buf) ){
		if (EAGAIN == errno) {
			return;
			errno_exit("EAGAIN");
		} else if (EINVAL == errno) {
			errno_exit("EINVAL");
		} else if (ENOMEM == errno) {
			errno_exit("ENOMEM");
		} else if (EIO == errno) {
			errno_exit("EIO");
		} else {
			errno_exit("VIDIOC_DQBUF");
		}
	}
	if (this->sequence != 0)
		assert(buf.sequence == this->sequence + 1);
	this->sequence = buf.sequence;
	
	struct buffer *pbuf = &buffers[buf.index];

	if (onread != NULL)
		onread( (uint8_t *)pbuf->start, buf);

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
