#include "slam.h"
#include <cmath>
using namespace std;
using namespace cv;

SLAM::SLAM()
{
	A = Mat::zeros(13, 13, CV_32FC1) ;
	int pos[3] = {0, 7, 10} ;
	for( int i=0 ; i<3 ; i++ )
		for( int j=0 ; j<3 ; j++ )
			for( int k=0 ; k<3 ; k++ )
				A.at<float>( pos[i]+j, pos[i]+k ) = 1 ;
	previous_t = 0 ;
	X = Mat::zeros( 13, 1, CV_32FC1 ) ;
}
void SLAM::predict( int64_t timestamp_ns )
{
	float dt ;
	float ccc, sss, scc, css, csc, scs, ccs, ssc ;
	float theta[3] ;
	float _x, _y, _z, _w ;
	Mat dqw, dq, domega, tmp ; 

	dt = (timestamp_ns - previous_t)*1e9 ;
	theta[0] = X.at<float>( 10, 0 ) ;
	theta[1] = X.at<float>( 10, 1 ) ;
	theta[2] = X.at<float>( 10, 2 ) ;
	ccc = cos( theta[0] )*cos( theta[1] )*cos( theta[2] ) ;
	sss = sin( theta[0] )*sin( theta[1] )*sin( theta[2] ) ;
	scc = sin( theta[0] )*cos( theta[1] )*cos( theta[2] ) ;
	css = cos( theta[0] )*sin( theta[1] )*sin( theta[2] ) ;
	csc = cos( theta[0] )*sin( theta[1] )*cos( theta[2] ) ;
	scs = sin( theta[0] )*cos( theta[1] )*sin( theta[2] ) ;
	ccs = cos( theta[0] )*cos( theta[1] )*sin( theta[2] ) ;
	ssc = sin( theta[0] )*sin( theta[1] )*cos( theta[2] ) ;
	
	_x = ccc+sss ;
	_y = scc-css ;
	_z = csc+scs ;
	_w = ccs-ssc ;

	generate_dqw( dqw, _x, _y, _z, _w ) ;
	generate_dq( dq,  X.at<float>( 3, 0 ),  X.at<float>( 4, 0 ),  X.at<float>( 5, 0 ),  X.at<float>( 6, 0 ) ) ;
	generate_domega( domega, ccc, sss, scc, css, csc, scs, ccs, ssc ) ;
	
	for( int i=0 ; i<3 ; i++ )
		A.at<float>( i, i+6 ) = dt ; 
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<4 ; j++ )
			A.at<float>( i+3, j+3 ) = dqw.at<float>(i, j) ;
	tmp = dq.mul( domega )*dt ;
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<3 ; j++ )
			A.at<float>( i+10, j+3 ) = dqw.at<float>(i, j) ;

}

void SLAM::generate_dqw( Mat &dqw, float _x, float _y, float _z, float _w )
{
	dqw = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dqw.at<float>(0, 1) = _w, dqw.at<float>(0, 1) = _z, dqw.at<float>(0, 1) = -_y, dqw.at<float>(0, 1) = _x ;
	dqw.at<float>(1, 1) = -_z, dqw.at<float>(1, 1) = _w, dqw.at<float>(1, 1) = _x, dqw.at<float>(1, 1) = _y ;
	dqw.at<float>(2, 1) = _y, dqw.at<float>(2, 1) = -_x, dqw.at<float>(2, 1) = _w, dqw.at<float>(2, 1) = _z ;
	dqw.at<float>(3, 1) = -_x, dqw.at<float>(3, 1) = -_y, dqw.at<float>(3, 1) = -_z, dqw.at<float>(3, 1) = _w ;
}

void SLAM::generate_dq( Mat &dq, float x, float y, float z, float w ) 
{
	dq = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dq.at<float>(0, 1) = w, dq.at<float>(0, 1) = -z, dq.at<float>(0, 1) = y, dq.at<float>(0, 1) = x ;
	dq.at<float>(1, 1) = z, dq.at<float>(1, 1) = w, dq.at<float>(1, 1) = -x, dq.at<float>(1, 1) = y ;
	dq.at<float>(2, 1) = -y, dq.at<float>(2, 1) = x, dq.at<float>(2, 1) = w, dq.at<float>(2, 1) = z ;
	dq.at<float>(3, 1) = -x, dq.at<float>(3, 1) = -y, dq.at<float>(3, 1) = -z, dq.at<float>(3, 1) = w ;
}

void SLAM::generate_domega( Mat &domega, float ccc, float sss, float scc, float css, float csc, float scs, float ccs, float ssc ) 
{
	domega = Mat::zeros( 4, 3, CV_32FC1 ) ;
	
}
