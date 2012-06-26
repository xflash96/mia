#include "convert.h"
#include "iothread.h"
#include "mia.h"

static gboolean
StereoThread_on_read(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return STEREO_THR->on_read(source, condition, data);
}
gboolean
StereoThread::on_read(GIOChannel *source, GIOCondition condition, gpointer data)
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
			if(cap==left_cap)
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

	if (cap == left_cap) {
		left_img = m;
	} else if (cap == right_cap) {
		right_img = m;
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

#if 0
	if(matched_count%100==0)
	calib_cameras_poses(CTX->left_img, CTX->right_img, CTX->hd_img,
			CTX->stereo, CTX->hd);
#endif
#if 1
	Pts3D feat_pts;
	cv::Mat left_descr, right_descr;
	stereo.get_feat_pts(left_img, right_img, feat_pts, left_descr, right_descr);
#else
	cv::imshow("left", left_img);
	cv::imshow("right", right_img);
	cv::moveWindow("left", 0, 0);
	cv::moveWindow("right", left_img.cols, 0);
	if(matched_count%100==0)
	calib_cameras_poses(left_img, right_img, HD_THR->hd_img, &stereo, &HD_THR->hd_cam);
#endif
#if 1
	struct GUIPacket *guip = new GUIPacket();
	guip->pts = feat_pts;
	GUI_THR->queue->push(guip);
#endif

	return TRUE;
}

static gboolean
StereoThread_on_msg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return STEREO_THR->on_msg(source, condition, data);
}
gboolean
StereoThread::on_msg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}

StereoThread::StereoThread(
	const char *left_cap_dev, const char *right_cap_dev,
	const char *left_rec_prefix, const char *right_rec_prefix,
	const char *intrinsics_path, const char *extrinsics_path)
{
	bool replay_mode = !(left_cap_dev && right_cap_dev);
	queue = new AsyncQueue(StereoThread_on_msg);
	V4LCaptureParam p = {
		width: 320, height: 240,
		fps: 100,
		pixelformat: V4L2_PIX_FMT_YUYV,
		record_prefix: NULL,
		replay_mode: (int)replay_mode,
	};
	p.record_prefix = left_rec_prefix;
	left_cap  = new V4LCapture(left_cap_dev, p);
	p.record_prefix = right_rec_prefix;
	right_cap = new V4LCapture(right_cap_dev, p);
	this->intrinsics_path = intrinsics_path;
	this->extrinsics_path = extrinsics_path;

	load_calib_params();
	if(!replay_mode){
		left_cap->start_capturing();
		right_cap->start_capturing();
		main_loop_add_fd (left_cap->fd, StereoThread_on_read, left_cap, G_PRIORITY_HIGH);
		main_loop_add_fd (right_cap->fd, StereoThread_on_read, right_cap, G_PRIORITY_HIGH);
	}
}

StereoThread::~StereoThread()
{
	delete left_cap;
	delete right_cap;
	delete queue;
}

void
StereoThread::load_calib_params()
{
	cv::FileStorage fs;
	fs = cv::FileStorage(intrinsics_path, cv::FileStorage::READ);
	stereo.left.load_intr(fs, "left_");
	stereo.right.load_intr(fs, "right_");
	stereo.load_F(fs);
	fs.release();
	fs = cv::FileStorage(extrinsics_path, cv::FileStorage::READ);
	stereo.left.load_extr(fs, "left_");
	stereo.right.load_extr(fs, "right_");
	fs.release();
}

void
HDVideoThread::load_calib_params()
{
	cv::FileStorage fs;
	fs = cv::FileStorage(intrinsics_path, cv::FileStorage::READ);
	hd_cam.load_intr(fs, "hd_");
	fs.release();
	fs = cv::FileStorage(extrinsics_path, cv::FileStorage::READ);
	hd_cam.load_extr(fs, "hd_");
	fs.release();
}

static gboolean
HDVideoThread_on_read(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return HD_THR->on_read(source, condition, data);
}
gboolean
HDVideoThread::on_read(GIOChannel *source, GIOCondition condition, gpointer data)
{
	assert (condition == G_IO_IN);

	/* Read frame */
	uint8_t *frame;
	struct v4l2_buffer buf;
	int ret;

	ret = hd_cap->read_frame(&frame, &buf);
	if(ret == 0)
		return FALSE;
	hd_decoder->read(frame, buf.bytesused);

	while(1){
		AVFrame* dframe = hd_decoder->decode();
		if(!dframe) break;
		cv::Mat m = avframe_to_cvmat(dframe);
		//imshow("hd", m);
		hd_img = m;
	}

	return TRUE;
}

static gboolean
HDVideoThread_on_msg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return HD_THR->on_msg(source, condition, data);
}
gboolean
HDVideoThread::on_msg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}

HDVideoThread::HDVideoThread(
	const char *hd_cap_dev, const char *hd_rec_prefix,
	const char *intrinsics_path, const char *extrinsics_path)
{
	bool replay_mode = !hd_cap_dev;
	queue = new AsyncQueue(HDVideoThread_on_msg);

	V4LCaptureParam p = {
		width: 800, height: 600,
		fps: 30,
		pixelformat: V4L2_PIX_FMT_H264,
		record_prefix: hd_rec_prefix,
		replay_mode: (int)replay_mode,
	};
	hd_cap = new V4LCapture(hd_cap_dev, p);
	hd_decoder = new FFStreamDecoder("h264");
	//HD_CAM->set_h264_video_profile();
	this->intrinsics_path = intrinsics_path;
	this->extrinsics_path = extrinsics_path;

	load_calib_params();
	if(!replay_mode){
		hd_cap->start_capturing();
		main_loop_add_fd (hd_cap->fd, HDVideoThread_on_read, NULL, G_PRIORITY_HIGH);
	}
}

HDVideoThread::~HDVideoThread()
{
	delete hd_cap;
	delete hd_decoder;
	delete queue;
}

int64_t start_time_diff = INT64_MAX;
gboolean
replay_timeout(gpointer data)
{
	int64_t min_time= INT64_MAX;
	GIOFunc min_callback = NULL;
	V4LCapture *min_cap = NULL;

	V4LCapture *caps[] = {
		STEREO_THR->left_cap,
		STEREO_THR->right_cap,
		HD_THR->hd_cap};
	GIOFunc callbacks[] = {
		StereoThread_on_read,
		StereoThread_on_read,
		HDVideoThread_on_read
	};
	int n_caps = 3;

	for(int i=0; i<n_caps; i++){
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

