#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

struct V4LCaptureParam
{
	int width, height;
	int fps;
	int pixelformat;
};

class V4LCapture
{
public:
	V4LCapture(const char *dev_name, struct V4LCaptureParam param, int (*process_image)(uint8_t *data, struct v4l2_buffer buf));
	~V4LCapture();

	char *dev_name;
	struct V4LCaptureParam param;
	int (*process_image)(uint8_t *data, struct v4l2_buffer buf);

	int fd;
	int n_buffers;
	__u32 sequence; /* v4l2_buffer.sequence */

	struct buffer {
		void *start;
		int length;
	} *buffers;

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
};

