#ifndef __SLAM_H__
#define __SLAM_H__
#define MAX_F 300 
#define X_dim 13

#include "cam.h"

class SLAM
{
public:
	SLAM() ; 
	void initial() ;
	void predict(int64_t timestamp_ns);
	void measure(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns);
private:
	cv::Mat A, X, X_tmp, r, y ;
	cv::Mat sigma, sigma_tmp ;
	cv::Mat Q, K, H, I, R_inv, nR_inv ;
	cv::Mat leftMap, rightMap ;
	Pts3D posMap ;
	cv::Mat H_y, sigma_ry, H_sigma_H, sigma_H, HX ;
	cv::Mat sigma_y, sigma_r ;
	int64_t previous_t ;

	void generate_dqw( cv::Mat &dqw, float _w, float _x, float _y, float _z ) ;
	void generate_dq( cv::Mat &dq, float w, float x, float y, float z) ;
	void generate_domega( cv::Mat &domega, float ccc, float sss, float scc, float css, float csc, float scs, float ccs, float ssc ) ;
	void generateR( cv::Mat &R, float q0, float q1, float q2, float q3 ) ;
	void generate_HsigmaH( cv::Mat &H_sigma_H, int idx, cv::Mat &R_inv, cv::Mat &nR_inv, cv::Mat &sigma ) ;
	void generate_sigmaH( cv::Mat &sigma_H, int idx, cv::Mat &sigma, cv::Mat &R_inv ) ;
	void generate_HX( cv::Mat &HX, int idx, cv::Mat &R_inv ) ;
};

#endif
