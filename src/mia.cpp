#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <glib.h>
#include <gtk/gtk.h>

#include "cap.h"
#include "decoder.h"

V4LCapture *STEREO_LEFT_CAM, *STEREO_RIGHT_CAM, *HD_CAM;
FFStreamDecoder *HD_DECODER;

GAsyncQueue *STEREO_MSG_Q, *HDVIDEO_MSG_Q, *UI_MSG_Q;
int stereo_msg_pipe[2], hdvideo_msg_pipe[2], ui_msg_pipe[2];

const char *LEFT_CAM_DEV;
const char *RIGHT_CAM_DEV; 
const char *HD_CAM_DEV;
const char *LEFT_CAM_RECORD_PREFIX;
const char *RIGHT_CAM_RECORD_PREFIX;
const char *HD_CAM_RECORD_PREFIX;

bool replay_mode = false;
bool record_video = false;

void errno_exit(const char *tmpl, ...)
{
	va_list ap;
	va_start(ap, tmpl);
	vfprintf(stderr, tmpl, ap);
	fprintf(stderr, "\n\terror %d: %s\n", errno, strerror(errno));
	va_end(ap);
	exit(EXIT_FAILURE);
}

void main_loop_add_fd(int fd, GIOFunc callback, gpointer data)
{
	GIOChannel *chan = NULL;
	chan = g_io_channel_unix_new(fd);
	g_io_add_watch(chan, G_IO_IN, callback, data);
}

void signal_pipe(int *fds)
{
	write(fds[1], "1", 1);
}

/* Stereo Vision CallBacks
 *
 */
static gboolean
stereo_onread(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	V4LCapture *cap = (V4LCapture *)data;
	int ret = cap->read_frame(NULL);
	if(ret == 0)
		return FALSE;
	if (cap == STEREO_LEFT_CAM) {
	} else if (cap == STEREO_RIGHT_CAM) {
	} else {
		errno_exit("NOT proper CAM");
	}

	return TRUE;
}

static gboolean
stereo_timeout(gpointer data)
{
	return stereo_onread(NULL, G_IO_IN, data);
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
		width: 640, height: 480,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_YUYV,
		record_prefix: NULL,
		replay_mode: (int)replay_mode,
	};
	p.record_prefix = LEFT_CAM_RECORD_PREFIX;
	STEREO_LEFT_CAM  = new V4LCapture(LEFT_CAM_DEV, p);
	p.record_prefix = RIGHT_CAM_RECORD_PREFIX;
	STEREO_RIGHT_CAM = new V4LCapture(RIGHT_CAM_DEV, p);

	main_loop_add_fd (stereo_msg_pipe[0],   stereo_onmsg, NULL);
	if(replay_mode){
		g_timeout_add(1000/p.fps, stereo_timeout, STEREO_LEFT_CAM);
		g_timeout_add(1000/p.fps, stereo_timeout, STEREO_RIGHT_CAM);
	}else{
		main_loop_add_fd (STEREO_LEFT_CAM->fd, stereo_onread, STEREO_LEFT_CAM);
		main_loop_add_fd (STEREO_RIGHT_CAM->fd, stereo_onread, STEREO_RIGHT_CAM);
	}
}

static void 
stereo_destroy()
{
	g_async_queue_unref(STEREO_MSG_Q);
}


/* HD Video CallBacks
 *
 */
int
hdvideo_on_frame(AVFrame *frame)
{
	return 0;
}
int
hdvideo_decode(uint8_t *data, struct v4l2_buffer *buf)
{
	HD_DECODER->decode(data, buf->bytesused, hdvideo_on_frame);
	return 0;
}
gboolean
hdvideo_onread(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	int ret = HD_CAM->read_frame(hdvideo_decode);
	if(ret == 0)
		return FALSE;


	return TRUE;
}
static gboolean
hdvideo_timeout(gpointer data)
{
	return hdvideo_onread(NULL, G_IO_IN, data);
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
		width: 1920, height: 1080,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_H264,
		record_prefix: HD_CAM_RECORD_PREFIX,
		replay_mode: (int)replay_mode,
	};
	HD_CAM = new V4LCapture(HD_CAM_DEV, p);
	HD_DECODER = new FFStreamDecoder("h264");

	main_loop_add_fd (hdvideo_msg_pipe[0],  hdvideo_onmsg, NULL);
	if(replay_mode){
		g_timeout_add (1000/p.fps, hdvideo_timeout, NULL);
	}else{
		main_loop_add_fd (HD_CAM->fd, hdvideo_onread, NULL);
	}
}

static void
hdvideo_destroy()
{
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
	main_loop_add_fd(ui_msg_pipe[0], ui_onmsg, NULL);
}


static void end_program(void)
{
	stereo_destroy();
	hdvideo_destroy();
	//gtk_main_quit();
	exit(0);
}

int main(int argc, char **argv)
{
	avcodec_register_all();
	av_register_all();
	av_log_set_level(AV_LOG_DEBUG);

	replay_mode = true;
	LEFT_CAM_DEV  = "/dev/video1";
	RIGHT_CAM_DEV = "/dev/video0";
	HD_CAM_DEV = "/dev/video2";
	LEFT_CAM_RECORD_PREFIX = "data/rec_left";
	RIGHT_CAM_RECORD_PREFIX = "data/rec_right";
	HD_CAM_RECORD_PREFIX = "data/rec_hd";

	GMainLoop* main_loop = NULL;
	main_loop = g_main_loop_new (NULL, FALSE);

	g_thread_init(NULL);
	/* Queue INIT */
	stereo_init();
	hdvideo_init();
	ui_init();

	/* GTK INIT */
	GtkWidget *window;
	gtk_init (&argc, &argv);
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	g_signal_connect (window, "destroy", G_CALLBACK (end_program), NULL);
	gtk_widget_show (window);

	g_main_loop_run(main_loop);
	return 0;
}

