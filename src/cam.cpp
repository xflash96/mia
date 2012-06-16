#include <stdio.h>
#include "cam.h"

using namespace cv;

void Cam::undistortPoints(Pts2D &ipoints, Pts2D &opoints)
{
	cv::undistortPoints(ipoints, opoints, cam_mat, dist_coeff);
}

void Cam::projectPoints(Pts3D &ipoints, Pts2D &opoints)
{
	cv::projectPoints(ipoints, rvec, tvec, cam_mat, dist_coeff, opoints);
}

bool Cam::solvePnP(Pts3D &objPoints, Pts2D &imgPoints)
{
	bool ok = cv::solvePnP(objPoints, imgPoints,
			cam_mat, dist_coeff, rvec, tvec, false, CV_EPNP);
	return ok;
}

void Cam::load_intr(FileStorage &fs, const char *prefix)
{
	char name[100];
	if(prefix==NULL)
		prefix = "";
	sprintf(name, "%scam_mat", prefix);
	fs[name] >> cam_mat;
	sprintf(name, "%sdist_coeff", prefix);
	fs[name] >> dist_coeff;
}
void Cam::dump_intr(FileStorage &fs, const char *prefix)
{
	char name[100];
	if(prefix==NULL)
		prefix = "";
	sprintf(name, "%scam_mat", prefix);
	fs << name << cam_mat;
	sprintf(name, "%sdist_coeff", prefix);
	fs << name << dist_coeff;
}
void Cam::load_extr(FileStorage &fs, const char *prefix)
{
	char name[100];
	if(prefix==NULL)
		prefix = "";
	sprintf(name, "%sproj", prefix);
	fs[name] >> cam_mat;
	sprintf(name, "%srvec", prefix);
	fs[name] >> rvec;
	sprintf(name, "%stvec", prefix);
	fs[name] >> tvec;
}
void Cam::dump_extr(FileStorage &fs, const char *prefix)
{
	char name[100];
	if(prefix==NULL)
		prefix = "";
	sprintf(name, "%sproj", prefix);
	fs << name << proj;
	sprintf(name, "%srvec", prefix);
	fs << name << rvec;
	sprintf(name, "%stvec", prefix);
	fs << name << tvec;
}
