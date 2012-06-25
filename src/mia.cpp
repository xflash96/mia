#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <glib.h>

#include <math.h>

#include "mia.h"

int64_t process_start_time = INT64_MAX;

bool replay_mode = false;
const char *LEFT_CAM_DEV = NULL;
const char *RIGHT_CAM_DEV = NULL; 
const char *HD_CAM_DEV = NULL;

const char *LEFT_REC_PREFIX;
const char *RIGHT_REC_PREFIX;
const char *HD_REC_PREFIX;

const char *INTRINSICS_PATH;
const char *EXTRINSICS_PATH;

StereoThread *STEREO_THR;
HDVideoThread *HD_THR;
GUI *GUI_THR;

void end_program(void)
{
	//gtk_main_quit();
	if(STEREO_THR)
		delete STEREO_THR;
	if(HD_THR)
		delete HD_THR;
	if(GUI_THR)
		delete GUI_THR;
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

	//CTX = new MiaContext();
	replay_mode = argc<2;

	if(!replay_mode){
		LEFT_CAM_DEV  	= "/dev/video0";
		RIGHT_CAM_DEV 	= "/dev/video1";
		HD_CAM_DEV 	= "/dev/video2";
	}
	LEFT_REC_PREFIX 	= "data/rec_left";
	RIGHT_REC_PREFIX 	= "data/rec_right";
	HD_REC_PREFIX 		= "data/rec_hd";

	INTRINSICS_PATH		= "intrinsics.yml";
	EXTRINSICS_PATH 	= "extrinsics.yml";

	GMainLoop* main_loop = NULL;
	main_loop = g_main_loop_new (NULL, FALSE);

	gtk_init (&argc, &argv);
	gtk_gl_init (&argc, &argv);
	g_thread_init(NULL);

	STEREO_THR = new StereoThread(
			LEFT_CAM_DEV, RIGHT_CAM_DEV,
			LEFT_REC_PREFIX, RIGHT_REC_PREFIX,
			INTRINSICS_PATH, EXTRINSICS_PATH);

	HD_THR = new HDVideoThread(
			HD_CAM_DEV, HD_REC_PREFIX,
			INTRINSICS_PATH, EXTRINSICS_PATH);

//	GUI_THR = new GUI();

	signal(SIGINT, sigint_handler);
	signal(SIGTERM, sigint_handler);

	if(replay_mode)
		g_timeout_add_full(G_PRIORITY_HIGH, 2, replay_timeout, NULL, NULL);
	else
		process_start_time = time_now_ns();

	fprintf(stderr, "Finished initializing\n");
	g_main_loop_run(main_loop);
	return 0;
}

