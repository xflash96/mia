#ifndef __SLAM_H__
#define __SLAM_H__

#include "cam.h"

class SLAM
{
public:
	SLAM() 
	void predict(int64_t timestamp_ns);
	void update(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns);
private:
	cv::Mat A ;
	cv::Mat H ;
	cv::Mat X ;
	cv::Mat X_hat ;
	int64_t previous_t ;

	void generate_dqw( cv::Mat &dqw, float _x, float _y, float _z, float _w ) ;
	void generate_dq( cv::Mat &dq, float x, float y, float z, float w ) ;
	void generate_domega( cv::Mat &domega, float ccc, float sss, float scc, float css, float csc, float scs, float ccs, float ssc ) ;
};

#endif
