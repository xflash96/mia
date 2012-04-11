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

#include <opencv2/core/core_c.h>
#include <opencv2/opencv.hpp>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class V4LCapture
{
public:
	V4LCapture(const char *dev_name) { 
		this->dev_name = strdup(dev_name); 
		width = 320, height = 240;
		n_buffers = 4;
		fps = 120;
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
	IplImage frame;
	int width, height;
	int fps;
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

	void write_ppm(const char* name);
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
void V4LCapture::write_ppm(const char *name)
{
	FILE *fp = fopen(name, "w");
	fprintf(fp, "P6\n%d %d 255\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
	fwrite(frame.imageData, frame.imageSize, 1, fp);
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

	cvInitImageHeader( &frame,
			   cvSize( fmt.fmt.pix.width,
				   fmt.fmt.pix.height ),
			   IPL_DEPTH_8U, 3, IPL_ORIGIN_TL, 4 );
	frame.imageData = (char*)cvAlloc(frame.imageSize);
	if (NULL==frame.imageData){
		errno_exit("calloc");
	}

	struct v4l2_streamparm setfps;
	
	CLEAR (setfps);

	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps.parm.capture.timeperframe.numerator = 1;
	setfps.parm.capture.timeperframe.denominator = fps;
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
void V4LCapture::read_frame()
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
	
	struct buffer* pbuf = &buffers[buf.index];

	pbuf->bytesused = buf.bytesused;
	pbuf->no = count++;

	v4lconvert_yuyv_to_bgr24( 	(unsigned char*)pbuf->start, 
					(unsigned char*)frame.imageData,
				fmt.fmt.pix.width, fmt.fmt.pix.height);

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
	cvFree(&frame.imageData);
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

	for (int i=0; ; i++){
		r = select(cap.fd+1, &fds, NULL, NULL, &tv);
		if (r==-1) {
			errno_exit("select overtime");
		}
		cap.read_frame();
#if 0
	char out_name[20];
	sprintf(out_name, "out%d.ppm", pbuf->no);
	write_ppm(out_name);
#else
	cv::Mat mat = cv::Mat(&cap.frame, false);
	cv::imshow("view", mat);
	cvWaitKey(1);
#endif
		//fputc('.', stderr);
	}
	
	return 0;
}
