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

V4LCapture *STEREO_LEFT_CAM, *STEREO_RIGHT_CAM, *HD_CAM;

GAsyncQueue *STEREO_MSG_Q, *HDVIDEO_MSG_Q, *UI_MSG_Q;
int stereo_msg_pipe[2], hdvideo_msg_pipe[2], ui_msg_pipe[2];

void errno_exit(const char *tmpl, ...)
{
	va_list ap;
	va_start(ap, tmpl);
	vfprintf(stderr, tmpl, ap);
	fprintf(stderr, "\terror %d, %s\n", errno, strerror(errno));
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
	cap->read_frame(NULL);
	if (cap == STEREO_LEFT_CAM) {
	} else if (cap == STEREO_RIGHT_CAM) {
	} else {
		errno_exit("NOT proper CAM");
	}

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
		width: 640, height: 480,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_YUV420 
	};
	STEREO_LEFT_CAM  = new V4LCapture("", p);
	STEREO_RIGHT_CAM = new V4LCapture("", p);

	main_loop_add_fd (stereo_msg_pipe[0],   stereo_onmsg, NULL);
	main_loop_add_fd (STEREO_LEFT_CAM->fd,  stereo_onread, STEREO_LEFT_CAM);
	main_loop_add_fd (STEREO_RIGHT_CAM->fd, stereo_onread, STEREO_RIGHT_CAM);
}

static void 
stereo_destroy()
{
	g_async_queue_unref(STEREO_MSG_Q);
}


/* HD Video CallBacks
 *
 */
static gboolean
hdvideo_onread(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	HD_CAM->read_frame(NULL);

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
		width: 1920, height: 1080,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_H264,
	};
	HD_CAM = new V4LCapture("", p);

	main_loop_add_fd (hdvideo_msg_pipe[0],  hdvideo_onmsg, NULL);
	main_loop_add_fd (HD_CAM->fd, 		hdvideo_onread, NULL);
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

