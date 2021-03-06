#ifndef __SLAM_H__
#define __SLAM_H__
#define MAX_F 4
#define X_dim 13

#include <opencv2/opencv.hpp>
#include "cam.h"

class SLAM
{
public:
	SLAM() ; 
	~SLAM() ;
	void initial( Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns ) ;
	void predict(int64_t timestamp_ns);
	void measure(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns);
	void feature( Pts3D &positions, Pts3D &variance ) ;
//private:
	cv::Mat A, X, X_tmp, r, y, q ;
	cv::Mat sigma, sigma_tmp ;
	cv::Mat R, Q ;
	cv::Mat K, H, I, R_inv, nR_inv, R_dq ;
	cv::Mat leftMap, rightMap ;
	cv::Mat dqw, dq, domega ;
	Pts3D posMap ;
	cv::Mat H_y, sigma_ry, H_sigma_H, sigma_H, HX ;
	cv::Mat sigma_y, sigma_r, sigma_Rq ;
	//cv::DescriptorMatcher *matcher ;
	cv::BFMatcher *matcher ;
	int y_size ;
	int *left_matchList, *right_matchList ;
	float epsilon ;


	int64_t previous_t ;

	void generate_dq( cv::Mat &dq, float _w, float _x, float _y, float _z ) ;
	void generate_dqw( cv::Mat &dqw, float w, float x, float y, float z) ;
	void generate_domega( cv::Mat &domega, float ccc, float sss, float scc, float css, float csc, float scs, float ccs, float ssc ) ;
	void generateR( cv::Mat &R, float q0, float q1, float q2, float q3 ) ;
	void generate_R_dq( float a, float b, float c, float q0, float q1, float q2, float q3 ) ;
	void generate_HsigmaH( cv::Mat &H_sigma_H, int idx, cv::Mat &R_inv, cv::Mat &nR_inv, cv::Mat &sigma ) ;
	void generate_sigmaH( cv::Mat &sigma_H, int idx, cv::Mat &sigma, cv::Mat &R_inv ) ;
	void generate_HX( cv::Mat &HX, int idx, cv::Mat &R_inv ) ;
};

#endif
