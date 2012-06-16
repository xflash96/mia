#ifndef __SLAM_H__
#define __SLAM_H__

#include "cam.h"

class SLAM
{
public:
	void predict(int64_t timestamp_ns);
	void update(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns);
};

#endif
