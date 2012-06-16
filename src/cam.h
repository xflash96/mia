#ifndef __CAM_H__
#define __CAM_H__

#include <opencv2/opencv.hpp>

using namespace std;

typedef vector<cv::Point2f> Pts2D;
typedef vector<cv::Point3f> Pts3D;

class Cam{
public:
	cv::Mat cam_mat, dist_coeff; 	//intrinsics
	cv::Mat proj, rvec, tvec; 	//extrinsics

	void undistortPoints(Pts2D &ipoints, Pts2D &opoints);
	void projectPoints(Pts3D &ipoints, Pts2D &opoints);
	bool solvePnP(Pts3D &objPoints, Pts2D &imgPoints);
	
	void load_intr(cv::FileStorage &fs, const char *prefix=NULL);
	void dump_intr(cv::FileStorage &fs, const char *prefix=NULL);
	void load_extr(cv::FileStorage &fs, const char *prefix=NULL);
	void dump_extr(cv::FileStorage &fs, const char *prefix=NULL);
};

#endif
