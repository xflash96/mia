#ifndef __TEST_H__
#define __TEST_H__

#include "slam.h"
#include "cam.h"
class SLAMTest{
public:
	SLAMTest();
	float dt;
	SLAM slam;
	cv::Mat x, rvec, v, omega, M, descr;
	float noise;
	int iter;

	void run_camera_once();
};

#endif
