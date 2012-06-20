#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
# ifndef __STDC_LIMIT_MACROS
# define __STDC_LIMIT_MACROS
# endif
#include <stdint.h>
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <glib.h>
#include <gtk/gtk.h>

#include "stereo.h"
#include "cap.h"
#include "decoder.h"
#include "convert.h"
#include "calib.h"

class MiaContext
{
public:
	enum State
	{
		CALIB_STEREO,
		CALIB_HDVIDEO,
		ONLINE,
		REPLAY
	}state;

	bool record_video;

	V4LCapture *left_cam, *right_cam, *hd_cam;
	FFStreamDecoder *hd_decoder;

	cv::Mat left_img, right_img;
	cv::Mat hd_img;

	Stereo stereo;
	Cam hd;

	const char *intrinsics_path;
	const char *extrinsics_path;
} *CTX;

//static gint IO_PRIORITY = G_PRIORITY_HIGH;

const char *LEFT_CAM_DEV;
const char *RIGHT_CAM_DEV; 
const char *HD_CAM_DEV;

const char *LEFT_CAM_RECORD_PREFIX;
const char *RIGHT_CAM_RECORD_PREFIX;
const char *HD_CAM_RECORD_PREFIX;

const char *INTRINSICS_PATH;
const char *EXTRINSICS_PATH;

volatile int64_t process_start_time = INT64_MAX;
volatile int64_t stereo_prev_cap_time;
static bool replay_mode = false;

GAsyncQueue *STEREO_MSG_Q, *HDVIDEO_MSG_Q, *UI_MSG_Q;
int stereo_msg_pipe[2], hdvideo_msg_pipe[2], ui_msg_pipe[2];

void errno_exit(const char *tmpl, ...)
{
	va_list ap;
	va_start(ap, tmpl);
	vfprintf(stderr, tmpl, ap);
	fprintf(stderr, "\n\terror %d: %s\n", errno, strerror(errno));
	va_end(ap);
	exit(EXIT_FAILURE);
}

void main_loop_add_fd(int fd, GIOFunc callback, gpointer data, gint priority)
{
	GIOChannel *chan = NULL;
	chan = g_io_channel_unix_new(fd);
	g_io_add_watch_full(chan, priority, G_IO_IN, callback, data, NULL);
}

void signal_pipe(int *fds)
{
	write(fds[1], "1", 1);
}

int matched_count;
int total_count;
int STEREO_WAIT_PAIR;
/* Stereo Vision CallBacks
 *
 */
gboolean
stereo_onread(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	V4LCapture *cap = (V4LCapture *)data;
	uint8_t *frame;
	struct v4l2_buffer buf;
	int ret;
	int64_t tolerance= 1000000000L/cap->param.fps/2, cap_time;

	ret = cap->read_frame(&frame, &buf);
	if(ret == 0)
		return FALSE;
	total_count++;

	cap_time = timeval_to_ns(&buf.timestamp);
	if(cap_time > stereo_prev_cap_time+tolerance){
		// new frame comes, replace old whether it is matched
		if(STEREO_WAIT_PAIR){
			fprintf(stderr, "prev: %ld ", 
				(stereo_prev_cap_time-cap_time)/1000000);
			if(cap==CTX->left_cam)
				fprintf(stderr, "left\n");
			else
				fprintf(stderr, "right\n");
		}
		stereo_prev_cap_time = cap_time;
		STEREO_WAIT_PAIR = 1;
	}else if(cap_time < stereo_prev_cap_time-tolerance){
		// this frame is too old, disgard it
		return TRUE;
	}else{
		// frame matched, process pair
		STEREO_WAIT_PAIR = 0;
	}

	// conversion and save
	cv::Mat m = raw_to_cvmat(frame,
			cap->param.width, cap->param.height,
			PIX_FMT_YUYV422);

	if (cap == CTX->left_cam) {
		CTX->left_img = m;
	} else if (cap == CTX->right_cam) {
		CTX->right_img = m;
	} else {
		errno_exit("NOT proper CAM");
	}
	if(STEREO_WAIT_PAIR){
		return TRUE;
	}
	matched_count++;
	if(matched_count%100==0)
	fprintf(stderr, "mis: %d\t[%.3f]\n", 
		(total_count-matched_count*2),
		matched_count*1e9/(time_now_ns()-process_start_time));

	//cv::imshow("left", CTX->left_img);
	//cv::imshow("right", CTX->right_img);
	cv::moveWindow("left", 0, 0);
	cv::moveWindow("right", CTX->left_img.cols, 0);
#if 0
	if(matched_count%100==0)
	calib_cameras_poses(CTX->left_img, CTX->right_img, CTX->hd_img,
			CTX->stereo, CTX->hd);
#else
	process_stereo(CTX->left_img, CTX->right_img, CTX->stereo, cap_time);
#endif

	return TRUE;
}

static gboolean
stereo_onmsg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}

static void
stereo_init()
{
	STEREO_MSG_Q = g_async_queue_new();
	if (0 > pipe(stereo_msg_pipe)) {
		errno_exit("PIPE");
	}
	V4LCaptureParam p = {
		width: 320, height: 240,
		fps: 100,
		pixelformat: V4L2_PIX_FMT_YUYV,
		record_prefix: NULL,
		replay_mode: (int)replay_mode,
	};
	p.record_prefix = LEFT_CAM_RECORD_PREFIX;
	CTX->left_cam  = new V4LCapture(LEFT_CAM_DEV, p);
	p.record_prefix = RIGHT_CAM_RECORD_PREFIX;
	CTX->right_cam = new V4LCapture(RIGHT_CAM_DEV, p);

	main_loop_add_fd (stereo_msg_pipe[0],   stereo_onmsg, NULL, G_PRIORITY_HIGH);
	if(!replay_mode){
		CTX->left_cam->start_capturing();
		CTX->right_cam->start_capturing();
		main_loop_add_fd (CTX->left_cam->fd, stereo_onread, CTX->left_cam, G_PRIORITY_HIGH);
		main_loop_add_fd (CTX->right_cam->fd, stereo_onread, CTX->right_cam, G_PRIORITY_HIGH);
	}
}

static void 
stereo_destroy()
{
	delete CTX->left_cam;
	delete CTX->right_cam;
	g_async_queue_unref(STEREO_MSG_Q);
}

static void
load_calib_params(MiaContext *CTX)
{
	cv::FileStorage fs;
	fs = cv::FileStorage(CTX->intrinsics_path, cv::FileStorage::READ);
	CTX->stereo.left.load_intr(fs, "left_");
	CTX->stereo.right.load_intr(fs, "right_");
	CTX->hd.load_intr(fs, "hd_");
	fs.release();
	fs = cv::FileStorage(CTX->extrinsics_path, cv::FileStorage::READ);
	CTX->stereo.left.load_extr(fs, "left_");
	CTX->stereo.right.load_extr(fs, "right_");
	CTX->hd.load_extr(fs, "hd_");
	fs.release();
}

int hd_count = 0;
/* HD Video CallBacks
 *
 */
gboolean
hdvideo_onread(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	uint8_t *frame;
	struct v4l2_buffer buf;
	int ret;

	ret = CTX->hd_cam->read_frame(&frame, &buf);
	if(ret == 0)
		return FALSE;
	CTX->hd_decoder->read(frame, buf.bytesused);

	while(1){
		AVFrame* dframe = CTX->hd_decoder->decode();
		if(!dframe) break;
		cv::Mat m = avframe_to_cvmat(dframe);
		//imshow("hd", m);
		CTX->hd_img = m;
	}

	return TRUE;
}

static gboolean
hdvideo_onmsg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}

static void
hdvideo_init()
{
	HDVIDEO_MSG_Q = g_async_queue_new();
	if (0 > pipe(hdvideo_msg_pipe)) {
		errno_exit("PIPE");
	}
	V4LCaptureParam p = {
		width: 800, height: 600,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_H264,
		record_prefix: HD_CAM_RECORD_PREFIX,
		replay_mode: (int)replay_mode,
	};
	CTX->hd_cam = new V4LCapture(HD_CAM_DEV, p);
	CTX->hd_decoder = new FFStreamDecoder("h264");
	//HD_CAM->set_h264_video_profile();

	main_loop_add_fd (hdvideo_msg_pipe[0],  hdvideo_onmsg, NULL, G_PRIORITY_HIGH);
	if(!replay_mode){
		CTX->hd_cam->start_capturing();
		main_loop_add_fd (CTX->hd_cam->fd, hdvideo_onread, NULL, G_PRIORITY_HIGH);
	}
}

static void
hdvideo_destroy()
{
	delete CTX->hd_cam;
	g_async_queue_unref(HDVIDEO_MSG_Q);
}


/* UI CallBacks
 *
 */
static gboolean
ui_onmsg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}

static void
ui_init()
{
	UI_MSG_Q = g_async_queue_new();
	if (0 > pipe(ui_msg_pipe)) {
		errno_exit("PIPE");
	}
	main_loop_add_fd(ui_msg_pipe[0], ui_onmsg, NULL, G_PRIORITY_DEFAULT);
}

int64_t start_time_diff = INT64_MAX;
gboolean
replay_timeout(gpointer data)
{
	int64_t min_time= INT64_MAX;
	GIOFunc min_callback = NULL;
	V4LCapture *min_cap = NULL;

	V4LCapture *caps[] = {
		CTX->left_cam,
		CTX->right_cam,
		CTX->hd_cam };
	GIOFunc callbacks[] = {
		stereo_onread,
		stereo_onread,
		hdvideo_onread,
	};
	int n_cams = 3;

	for(int i=0; i<n_cams; i++){
		V4LCapture *cap = caps[i];
		int64_t cap_time = timeval_to_ns(&cap->buf_next.timestamp);
		if(min_time > cap_time){
			min_time = cap_time;
			min_callback = callbacks[i];
			min_cap = cap;
		}
	}

	if(start_time_diff == INT64_MAX){
		start_time_diff = min_time - time_now_ns();
		process_start_time = time_now_ns();
	}

	int64_t replay_time = time_now_ns() + start_time_diff;
	if(min_time < replay_time){
		return min_callback(NULL, G_IO_IN, min_cap);
	}else{
		return TRUE;
	}
}

static void end_program(void)
{
	stereo_destroy();
	hdvideo_destroy();
	//gtk_main_quit();
	exit(0);
}

static void sigint_handler(int dummy)
{
	end_program();
}

int main(int argc, char **argv)
{
	avcodec_register_all();
	av_register_all();
	av_log_set_level(AV_LOG_DEBUG);

	CTX = new MiaContext();
	replay_mode = argc<2;
	LEFT_CAM_DEV  = "/dev/video0";
	RIGHT_CAM_DEV = "/dev/video1";
	HD_CAM_DEV = "/dev/video2";
	LEFT_CAM_RECORD_PREFIX = "data/rec_left";
	RIGHT_CAM_RECORD_PREFIX = "data/rec_right";
	HD_CAM_RECORD_PREFIX = "data/rec_hd";

	INTRINSICS_PATH= "intrinsics.yml";
	EXTRINSICS_PATH = "extrinsics.yml";
	CTX->intrinsics_path = INTRINSICS_PATH;
	CTX->extrinsics_path = EXTRINSICS_PATH;

	GMainLoop* main_loop = NULL;
	main_loop = g_main_loop_new (NULL, FALSE);

	g_thread_init(NULL);

	/* Queue INIT */
	stereo_init();
	hdvideo_init();
	ui_init();
	load_calib_params(CTX);

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);
	/* GTK INIT */
	GtkWidget *window;
	gtk_init (&argc, &argv);
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	g_signal_connect (window, "destroy", G_CALLBACK (end_program), NULL);
	//gtk_widget_show (window);

	if(replay_mode)
		g_timeout_add_full(G_PRIORITY_HIGH, 2, replay_timeout, NULL, NULL);
	else
		process_start_time = time_now_ns();

	g_main_loop_run(main_loop);
	return 0;
}

