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

#include "cap.h"
#include "convert.h"

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
	if(0==strcmp(dev_name, "")){
		this->dev_name = NULL;
		return;
	}
	this->dev_name = strdup(dev_name); 
	this->param = param;

	if(!param.replay_mode){
		struct timespec t;
		clock_gettime(CLOCK_MONOTONIC, &t);
		start_time = timespec_to_ns(&t);
	}
	// open records
	if(param.record_prefix!=NULL){
		const char *mode;
		char name[100];
		if(param.replay_mode)
			mode = "rb";
		else
			mode = "wb";
		sprintf(name, "%s.vid", param.record_prefix);
		video_rec = fopen(name, mode);
		if(!video_rec){
			errno_exit("cannot open file %s", name);
		}

		sprintf(name, "%s.time", param.record_prefix);
		time_rec = fopen(name, mode);
		if(!time_rec){
			errno_exit("cannot open file %s", name);
		}
		if(param.replay_mode){
			fread(&start_time, sizeof(start_time), 1, time_rec);
		}else{
			fwrite(&start_time, sizeof(start_time), 1, time_rec);
		}
	}

	buffers = NULL;
	if(param.replay_mode) {
		if(!param.record_prefix) {
			errno_exit("record_prefix empty while replay");
		}
		buffers = (struct buffer*) calloc(1, sizeof(*buffers));
		buffers->start = NULL;
		fread(&buf_next, sizeof(buf_next), 1, time_rec);
		return;
	}

	open_device();
	init_device();
	start_capturing();
};

V4LCapture::~V4LCapture() {
	if(param.replay_mode) {
		free(buffers->start);
		free(buffers);
	}else{
		stop_capturing();
		uninit_device();
		close_device(); 
	}
	if(!video_rec){
		fclose(video_rec);
	}
	if(!time_rec){
		fclose(time_rec);
	}
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
	fmt.fmt.pix.pixelformat	= param.pixelformat;
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

void V4LCapture::set_h264_video_profile()
{
	//FIXME see guvcview
	struct v4l2_queryctrl queryctrl;
	CLEAR(queryctrl);
	queryctrl.id = V4L2_CID_MPEG_CLASS;

	if(-1 == xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) 
              || queryctrl.flags & V4L2_CTRL_FLAG_DISABLED ){
		errno_exit("NOT SUPPORT MPEG EXT CTRL");
	}
	struct v4l2_ext_controls ext_ctrls;
	struct v4l2_ext_control ext_ctrl;

	ext_ctrl.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
	ext_ctrl.size = 0;
	ext_ctrl.value = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;

	ext_ctrls.ctrl_class = V4L2_CID_MPEG_CLASS;
	ext_ctrls.count = 1;
	ext_ctrls.controls = &ext_ctrl;

	if(-1 == xioctl(fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls)){
		errno_exit("VIDIOC_S_EXT_CTRLS");
	}
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

	// say it's emtpy
	buf_now.index = -1;
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

void V4LCapture::dump_frame(void *data, struct v4l2_buffer *buf)
{
	fwrite(data, buf->bytesused, 1, video_rec);
	fwrite(buf, sizeof(*buf), 1, time_rec);
#if 0
	fprintf(stderr, "dump %s %d\n", dev_name, buf->sequence);
#endif
}

int V4LCapture::load_frame(void **data, struct v4l2_buffer *buf)
{
	*buf = buf_next;
	if(*data == NULL){
		*data = malloc(buf->length);
		if(!*data){
			errno_exit("calloc error");
		}
	}
	int len = fread(*data, buf->bytesused, 1, video_rec);
#if 0
	fprintf(stderr, "load %s %d %d %d bytes\n", 
			dev_name, buf->sequence, buf->bytesused, buf->length);
#endif
	int ret = fread(&buf_next, sizeof(buf_next), 1, time_rec);
	if(ret == 0){
		buf_next.bytesused = 0;
	}
	return len;
}

int V4LCapture::read_frame(uint8_t **data, struct v4l2_buffer *buf)
{
	if (param.replay_mode) {
		int ret = load_frame(&(buffers->start), &buf_now);
		*data = (uint8_t *) buffers->start;
		*buf = buf_now;
		return ret;
	}

	if( (int)buf_now.index != -1 ) {
		if( -1 == xioctl(fd, VIDIOC_QBUF, &buf_now) ){
			errno_exit("VIDIOC_QBUF");
		}
	}
	
	CLEAR( buf_now );

	buf_now.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf_now.memory = V4L2_MEMORY_MMAP;

	if( -1 == xioctl(fd, VIDIOC_DQBUF, &buf_now) ){
		if (EAGAIN == errno) {
			return -1;
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
	
	struct buffer *pbuf = &buffers[buf_now.index];

	if (param.record_prefix != NULL) {
		dump_frame((uint8_t *)pbuf->start, &buf_now);
	}

	*data = (uint8_t *) pbuf->start;
	*buf = buf_now;

	return buf_now.bytesused;


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
