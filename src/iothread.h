#ifndef __IOTHREAD_H__
#define __IOTHREAD_H__

#include <glib.h>
#include "async.h"
#include "cap.h"
#include "stereo.h"
#include "decoder.h"

class StereoThread{
public:
	StereoThread(
		const char *left_cap_dev, const char *right_cap_dev,
		const char *left_rec_prefix, const char *right_rec_prefix,
		const char *intrinsics_path, const char *extrinsics_path);
	~StereoThread();
	AsyncQueue *queue;

	V4LCapture *left_cap, *right_cap;
	cv::Mat left_img, right_img;
	Stereo stereo;

	int64_t stereo_prev_cap_time;
	int matched_count;
	int total_count;
	int STEREO_WAIT_PAIR;
	const char *intrinsics_path, *extrinsics_path;

	gboolean
	on_read(GIOChannel *source, GIOCondition condition, gpointer data);
	gboolean
	on_msg(GIOChannel *source, GIOCondition condition, gpointer data);
	void load_calib_params();
};

class HDVideoThread{
public:
	HDVideoThread(
		const char *hd_cap_dev, const char *hd_rec_prefix,
		const char *intrinsics_path, const char *extrinsics_path);
	~HDVideoThread();
	AsyncQueue *queue;

	V4LCapture *hd_cap;
	Cam hd_cam;
	FFStreamDecoder *hd_decoder;
	cv::Mat hd_img;
	const char *intrinsics_path, *extrinsics_path;

	gboolean
	on_read(GIOChannel *source, GIOCondition condition, gpointer data);
	gboolean
	on_msg(GIOChannel *source, GIOCondition condition, gpointer data);
	void load_calib_params();
};

gboolean replay_timeout(gpointer data);
#endif
