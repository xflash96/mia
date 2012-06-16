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
	void triangulatePoints(Pts2D left_pts, Pts2D right_pts, Pts3D &dst_pts);
};
void process_stereo(int64_t cap_time);

#endif
