#include "stereo.h"

using namespace cv;

int ctr = 0;
void process_stereo(Mat left_img, Mat right_img, int64_t cap_time)
{
	vector<KeyPoint> left_keys, right_keys;
	Mat left_descrs, right_descrs;

#if 1
	int n_feature = 50;
	static ORB left_orb(n_feature, 1.2f, 1, 31, 0, 2, ORB::FAST_SCORE, 31);
	static ORB right_orb(n_feature, 1.2f, 1, 31, 0, 2, ORB::HARRIS_SCORE, 31);
	left_orb(left_img, Mat(), left_keys, left_descrs, false);
	right_orb(right_img, Mat(), right_keys, right_descrs, false);
#else

	Mat new_left, new_right;
	cvtColor(left_img, new_left, CV_RGB2GRAY);
	cvtColor(right_img, new_right, CV_RGB2GRAY);
	FAST(new_left, left_keys, 31, true);
	FAST(new_right, right_keys, 31, true);
#endif

	drawKeypoints(left_img, left_keys, left_img);
	drawKeypoints(right_img, right_keys, right_img);

	imshow("left_feat", left_img);
	imshow("right_feat", right_img);
//	waitKey(1);
}

void Stereo::triangulatePoints(Pts2D left_pts, Pts2D right_pts, Pts3D &dst_pts)
{
	Mat pts_4d;
	Pts2D left_udpts, right_udpts;

	left.undistortPoints(left_pts, left_udpts);
	right.undistortPoints(right_pts, right_udpts);
//	right_udpts = right_udpts.reshape(1, right_udpts.cols).t();
//	left_udpts = left_udpts.reshape(1, left_udpts.cols).t();

	cv::triangulatePoints(left.proj, right.proj, left_udpts, right_udpts, pts_4d);
	cv::convertPointsFromHomogeneous(pts_4d.t(), dst_pts);
}
