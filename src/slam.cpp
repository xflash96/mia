#include "slam.h"
#include <string.h>
#include <stdlib.h>
#include <cmath>
using namespace std;
using namespace cv;

SLAM::SLAM()
{
	A = Mat::zeros(X_dim, X_dim, CV_32FC1) ;
	int pos[3] = {0, 7, 10} ;
	for( int i=0 ; i<3 ; i++ )
		for( int j=0 ; j<3 ; j++ )
			for( int k=0 ; k<3 ; k++ )
				A.at<float>( pos[i]+j, pos[i]+k ) = 1 ;
	previous_t = 0 ;
	X = Mat::zeros( X_dim, 1, CV_32FC1 ) ;
}
void SLAM::initial()
{
	X = Mat::zeros( X_dim+3*MAX_F, 1, CV_32FC1 ) ;
	X.at<float>(3,0) = 1 ;
	sigma = Mat::eye( X_dim+3*MAX_F, X_dim+3*MAX_F, CV_32FC1 ) ;
	I = Mat::eye( X_dim+3*MAX_F, X_dim+3*MAX_F, CV_32FC1 ) ;
	H = Mat::zeros( 3, X_dim+3*MAX_F, CV_32FC1 ) ;
	y = Mat::zeros( 3, 1, CV_32FC1 ) ;
	r = Mat::zeros( 3, 1, CV_32FC1 ) ;

	//for H_sigma_H
	H_y = Mat::zeros( 3, X_dim+3, CV_32FC1 ) ;
	sigma_ry = Mat::zeros( X_dim+3, X_dim+3, CV_32FC1 ) ;
	sigma_r = Mat::zeros( 3, X_dim+3*MAX_F , CV_32FC1 ) ;
	sigma_y = Mat::zeros( 3, X_dim+3*MAX_F , CV_32FC1 ) ;
}
void SLAM::predict( int64_t timestamp_ns )
{
	float dt ;
	float ccc, sss, scc, css, csc, scs, ccs, ssc ;
	float theta[3] ;
	float _x, _y, _z, _w ;
	Mat dqw, dq, domega, tmp ; 

	dt = (timestamp_ns - previous_t)*1e9 ;
	theta[0] = X.at<float>( 10, 0 )*dt ;
	theta[1] = X.at<float>( 10, 1 )*dt ;
	theta[2] = X.at<float>( 10, 2 )*dt ;
	ccc = cos( theta[0] )*cos( theta[1] )*cos( theta[2] ) ;
	sss = sin( theta[0] )*sin( theta[1] )*sin( theta[2] ) ;
	scc = sin( theta[0] )*cos( theta[1] )*cos( theta[2] ) ;
	css = cos( theta[0] )*sin( theta[1] )*sin( theta[2] ) ;
	csc = cos( theta[0] )*sin( theta[1] )*cos( theta[2] ) ;
	scs = sin( theta[0] )*cos( theta[1] )*sin( theta[2] ) ;
	ccs = cos( theta[0] )*cos( theta[1] )*sin( theta[2] ) ;
	ssc = sin( theta[0] )*sin( theta[1] )*cos( theta[2] ) ;
	
	_w = ccc+sss ;
	_x = scc-css ;
	_y = csc+scs ;
	_z = ccs-ssc ;

	generate_dqw( dqw, _w, _x, _y, _z ) ;
	generate_dq( dq,  X.at<float>( 3, 0 ),  X.at<float>( 4, 0 ),  X.at<float>( 5, 0 ),  X.at<float>( 6, 0 ) ) ;
	generate_domega( domega, ccc, sss, scc, css, csc, scs, ccs, ssc ) ;
	
	for( int i=0 ; i<3 ; i++ )
		A.at<float>( i, i+6 ) = dt ; 
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<4 ; j++ )
			A.at<float>( i+3, j+3 ) = dqw.at<float>(i, j) ;
	tmp = dq*domega*dt ;
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<3 ; j++ )
			A.at<float>( i+10, j+3 ) = dqw.at<float>(i, j) ;
	memcpy( X_tmp.data, X.data, X_dim*sizeof(float) ) ;
	for( int i=0 ; i<X_dim ; i++ )
		memcpy( sigma_tmp.data+( sigma_tmp.cols*i*sizeof(float) ), sigma.data+( sigma.cols*i*sizeof(float)), 
		        sigma_tmp.cols*sizeof(float) ) ;
	X_tmp = A*X_tmp ;
	sigma_tmp = A*sigma_tmp*A.t() + Q ;

	memcpy( X.data, X_tmp.data, X_dim*sizeof(float) ) ;
	for( int i=0 ; i<X_dim ; i++ )
		memcpy( sigma.data+( sigma.cols*i*sizeof(float)), 
			sigma_tmp.data+( sigma_tmp.cols*i*sizeof(float) ), sigma_tmp.cols*sizeof(float) ) ;
}

void SLAM::measure(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns)
{
	int f_n = descrsLeft.rows ;
	int idx ;
	vector<int> matchList ;
	vector<int> addList ;
	/*match*/


	/*measure*/

	for( int _idx=0 ; _idx<matchList.size() ; _idx++ )	
	{
		generateR( R_inv, X.at<float>(3,0), X.at<float>(4,0), X.at<float>(5,0), X.at<float>(6,0) ) ;
		R_inv = R_inv.t() ;
		nR_inv = R_inv*(-1) ;
		//H
		for( int i=0 ; i<3 ; i++ )
			memcpy( H.data+(i*H.cols*sizeof(float)), nR_inv.data+(i*nR_inv.cols*sizeof(float)), 
			        nR_inv.cols*sizeof(float) ) ;
		idx = matchList[ _idx ] ;
		//generate H
		for( int i=0 ; i<3 ; i++ )
			memcpy( H.data+( (i*H.cols+X_dim+idx*3 ) *sizeof(float)), 
			        R_inv.data+(i*R_inv.cols*sizeof(float)), 
				R_inv.cols*sizeof(float) ) ;
		//H*sigma*H
		generate_HsigmaH( H_sigma_H, idx, R_inv, nR_inv, sigma ) ;
		generate_sigmaH( sigma_H, idx, sigma, R_inv ) ;
		//K = sigma*H*( H*sigma*H+R )^(-1) 
		K = sigma_H*( H_sigma_H ).inv() ;
		//generate HX
		generate_HX( HX, idx, R_inv ) ;
		memcpy( y.data, X.data+(X_dim+3*idx)*sizeof(float), 3*sizeof(float) ) ;
		X = X + K*( y-HX ) ;
		sigma = sigma-K*( H*sigma ) ;

		for( int i=0 ; i<3 ; i++ )
			memset( H.data+( (i*H.cols+X_dim+idx*3 ) *sizeof(float) ), 0, R_inv.cols*sizeof(float)  ) ;
	}
	//update
}

void SLAM::generate_dqw( Mat &dqw, float _w, float _x, float _y, float _z )
{
	dqw = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dqw.at<float>(0, 0) = _w, dqw.at<float>(0, 1) = _x, dqw.at<float>(0, 2) = -_y, dqw.at<float>(0, 3) = _z ;
	dqw.at<float>(1, 0) = -_x, dqw.at<float>(1, 1) = _w, dqw.at<float>(1, 2) = _z, dqw.at<float>(1, 3) = _y ;
	dqw.at<float>(2, 0) = _y, dqw.at<float>(2, 1) = -_z, dqw.at<float>(2, 2) = _w, dqw.at<float>(2, 3) = _x ;
	dqw.at<float>(3, 0) = -_z, dqw.at<float>(3, 1) = -_y, dqw.at<float>(3, 2) = -_x, dqw.at<float>(3, 3) = _w ;
}

void SLAM::generate_dq( Mat &dq, float w, float x, float y, float z ) 
{
	dq = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dq.at<float>(0, 0) = w, dq.at<float>(0, 1) = -x, dq.at<float>(0, 2) = -y, dq.at<float>(0, 3) = -z ;
	dq.at<float>(1, 0) = x, dq.at<float>(1, 1) = w, dq.at<float>(1, 2) = -z, dq.at<float>(1, 3) = y ;
	dq.at<float>(2, 0) = y, dq.at<float>(2, 1) = z, dq.at<float>(2, 2) = w, dq.at<float>(2, 3) = -x ;
	dq.at<float>(3, 0) = z, dq.at<float>(3, 1) = -y, dq.at<float>(3, 2) = z, dq.at<float>(3, 3) = w ;
}

void SLAM::generate_domega( Mat &domega, float ccc, float sss, float scc, float css, float csc, float scs, float ccs, float ssc ) 
{
	domega = Mat::zeros( 4, 3, CV_32FC1 ) ;
	domega.at<float>(0, 0) = -scc+css, domega.at<float>(0, 1) = -csc+scs, domega.at<float>(0, 2) = -ccs+ssc ;
	domega.at<float>(1, 0) = ccc+sss, domega.at<float>(1, 1) = -ssc-ccs, domega.at<float>(1, 2) = -scs-csc ;
	domega.at<float>(2, 0) = -ssc+ccs, domega.at<float>(2, 1) = ccc-sss, domega.at<float>(2, 2) = -css+scc ;
	domega.at<float>(3, 0) = -scs-css, domega.at<float>(3, 1) = -css-scc, domega.at<float>(3, 2) = ccc+sss ;
	domega = 0.5*domega ;
}

void SLAM::generate_HsigmaH( cv::Mat &H_sigma_H, int idx, cv::Mat &R_inv, cv::Mat &nR_inv, cv::Mat &sigma )
{
	//H_y
	for( int i=0 ; i<3 ; i++ )
	{
		memcpy( H_y.data+( i*H_y.cols*sizeof(float) ), 
			nR_inv.data+( i*nR_inv.cols*sizeof(float) ), nR_inv.cols*sizeof(float)  ) ;
		memcpy( H_y.data+( i*H_y.cols+X_dim )*sizeof(float), 
			R_inv.data+( i*R_inv.cols )*sizeof(float), ( i*R_inv.cols )*sizeof(float) ) ;
	}
	
	//sigma_ry
	for( int i=0 ; i<X_dim ; i++ )
	{
		memcpy( sigma_ry.data+(i*sigma_ry.cols)*sizeof(float), 
			sigma.data+(i*sigma.cols)*sizeof(float), X_dim*sizeof(float) ) ;
		memcpy( sigma_ry.data+(i*sigma_ry.cols+ X_dim )*sizeof(float), 
			sigma.data+(i*sigma.cols+ X_dim+3*idx )*sizeof(float), 3*sizeof(float) ) ;
	}
	for( int i=0 ; i<3 ; i++ )
	{
		int y_pos = X_dim+3*idx+i ;
		memcpy( sigma_ry.data+( (X_dim+i)*sigma_ry.cols )*sizeof(float), sigma.data+y_pos *sizeof(float),
			X_dim*sizeof(float) ) ;
		memcpy( sigma_ry.data+( (X_dim+i)*sigma_ry.cols+X_dim )*sizeof(float),
			sigma.data+( y_pos*sigma.cols + y_pos  )*sizeof(float), 3*sizeof(float) ) ;
	}
	H_sigma_H = H_y*sigma_ry*( H_y.t() ) ;
}

void SLAM::generate_sigmaH( cv::Mat &sigma_H, int idx, cv::Mat &sigma, cv::Mat &R_inv ) 
{
	for( int i=0 ; i<3 ; i++ )
	{
		memcpy( sigma_r.data+ i*sigma_r.cols*sizeof(float) , sigma.data+i*sigma.cols*sizeof(float), 
		        sigma.cols*sizeof(float) ) ;
		memcpy( sigma_y.data+ i*sigma_r.cols*sizeof(float) , sigma.data+(X_dim+idx*3+i )*sigma.cols*sizeof(float), 
		        sigma.cols*sizeof(float) ) ;
	}
	sigma_H = ( sigma_y-sigma_r ).t() * R_inv ;
}
	
void SLAM::generate_HX( cv::Mat &HX, int idx, cv::Mat &R_inv ) 
{
	memcpy( r.data, X.data, sizeof(float)*3 ) ;
	memcpy( y.data, X.data+(X_dim+idx*3)*sizeof(float) , sizeof(float)*3 ) ;
	HX = R_inv*( y-r ) ;
}
