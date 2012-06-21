#ifndef __STEREO_H___
#define __STEREO_H___

#include <opencv2/opencv.hpp>
extern "C"{
#include <stdint.h>
}
#include "cam.h"
using namespace std;

class Stereo
{
public:
	Cam left, right;
	cv::Mat F;
	cv::ORB *left_orb, *right_orb;
	cv::BFMatcher *matcher;

	Stereo();
	~Stereo();
	void triangulatePoints(Pts2D left_pts, Pts2D right_pts, Pts3D &dst_pts);
	void load_F(cv::FileStorage &fs);
	bool get_feat_pts(cv::Mat &left_img, cv::Mat &right_img, Pts3D &feat_pts, cv::Mat &left_descr, cv::Mat &right_descr);
};

#endif
