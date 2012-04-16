#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

struct buffer{
	void *	start;
	size_t	length;

	size_t bytesused;
	int no;
	struct v4l2_buffer buf;
};

class V4LCapture
{
public:
	V4LCapture(const char *dev_name);
	~V4LCapture();

	FILE *fdata, *findx;
	size_t offset;

	char *dev_name;
	int fd;
	struct buffer *buffers;
	IplImage frame;
	int width, height;
	int fps;
	int n_buffers;
	__u32 sequence; /* v4l2_buffer.sequence */

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

