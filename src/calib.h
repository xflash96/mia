#ifndef __CALIB_H__
#define __CALIB_H__

#include <opencv2/opencv.hpp>
#include "stereo.h"

bool calib_stereo_cameras(cv::Mat left_img, cv::Mat right_img, Cam left_cam, Cam right_cam);
bool calib_hd_camera(cv::Mat img, Cam cam);
bool calib_cameras_poses(cv::Mat left_img, cv::Mat right_img, cv::Mat hd_img,
		Stereo *stereo, Cam *hd_cam);

#endif
