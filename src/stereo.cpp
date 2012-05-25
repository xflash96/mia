#include "stereo.h"

using namespace std;
using namespace cv;

void process_stereo(cv::Mat left_img, cv::Mat right_img, int64_t cap_time)
{
	vector<KeyPoint> left_keys, right_keys;
	Mat left_descrs, right_descrs;
#if 1
	int n_feature = 200;
	ORB left_orb(n_feature);
	ORB right_orb(n_feature);

	left_orb(left_img, Mat(), left_keys, left_descrs, false);
	right_orb(right_img, Mat(), right_keys, right_descrs, false);
#else
	FAST(left_img, left_keys, 100);
	FAST(right_img, right_keys, 100);
#endif

	drawKeypoints(left_img, left_keys, left_img);
	drawKeypoints(right_img, right_keys, right_img);

	imshow("left_feat", left_img);
	imshow("right_feat", right_img);
}
