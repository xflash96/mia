#include "slam.h"
#include <string.h>
#include <stdlib.h>
#include <cmath>
using namespace std;
using namespace cv;

SLAM::SLAM() {}
SLAM::~SLAM()
{
	delete[] left_matchList ;
	delete[] right_matchList ;
	delete matcher ;
}
void SLAM::initial( Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns )
{
	A = Mat::eye(X_dim, X_dim, CV_32FC1) ;
	X_tmp = Mat::zeros( X_dim, X_dim, CV_32FC1 ) ;
	X = Mat::zeros( X_dim+3*MAX_F, 1, CV_32FC1 ) ;
	X.at<float>(3,0) = 1 ;
	sigma = Mat::eye( X_dim+3*MAX_F, X_dim+3*MAX_F, CV_32FC1 ) ;
	sigma_tmp = Mat::eye( X_dim, X_dim, CV_32FC1 ) ;
	I = Mat::eye( X_dim+3*MAX_F, X_dim+3*MAX_F, CV_32FC1 ) ;
	H = Mat::zeros( 3, X_dim+3*MAX_F, CV_32FC1 ) ;
	y = Mat::zeros( 3, 1, CV_32FC1 ) ;
	r = Mat::zeros( 3, 1, CV_32FC1 ) ;
	epsilon = 1e-3 ;
	R = epsilon*( Mat::eye( 3, 3, CV_32FC1 ) ) ;

	R_inv = Mat::zeros( 3, 3, CV_32FC1 ) ;
	previous_t = timestamp_ns ;

	//for H_sigma_H
	H_y = Mat::zeros( 3, 3+3, CV_32FC1 ) ;
	sigma_ry = Mat::zeros( 3+3, 3+3, CV_32FC1 ) ;
	sigma_r = Mat::zeros( 3, X_dim+3*MAX_F , CV_32FC1 ) ;
	sigma_y = Mat::zeros( 3, X_dim+3*MAX_F , CV_32FC1 ) ;
	//y_size = 0 ;
	left_matchList = new int[ MAX_F ] ;
	right_matchList = new int[ MAX_F ] ;
	leftMap = Mat::zeros( MAX_F, 32, CV_8UC1 ) ;
	rightMap = Mat::zeros( MAX_F, 32, CV_8UC1 ) ;
	matcher = new BFMatcher(NORM_HAMMING);

	for( int i=0 ; i<observedPoints.size() ; i++ )
	{
		memcpy( leftMap.data+i*32*sizeof(char), descrsLeft.data+i*32*sizeof(char), 32*sizeof(char) ) ;
		memcpy( rightMap.data+i*33*sizeof(char), descrsRight.data+i*32*sizeof(char), 32*sizeof(char) ) ;
		y.at<float>(0,0) = observedPoints[i].x ;
		y.at<float>(1,0) = observedPoints[i].y ;
		y.at<float>(2,0) = observedPoints[i].z ;
		memcpy( X.data+( X_dim+3*i )*sizeof(float), y.data, 3*sizeof(float) ) ;
		//y_size++ ;
	}
	y_size = observedPoints.size() ;
}
void SLAM::predict( int64_t timestamp_ns )
{
	float dt ;
	float ccc, sss, scc, css, csc, scs, ccs, ssc ;
	float theta[3] ;
	float _x, _y, _z, _w ;
	Mat dqw, dq, domega, tmp ; 

	dt = (timestamp_ns - previous_t)*1e-9 ;
	theta[0] = X.at<float>( 10, 0 )*dt*0.5 ;
	theta[1] = X.at<float>( 11, 0 )*dt*0.5 ;
	theta[2] = X.at<float>( 12, 0 )*dt*0.5 ;
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
		A.at<float>( i, i+7 ) = dt ; 
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<4 ; j++ )
			A.at<float>( i+3, j+3 ) = dqw.at<float>(i, j) ;
	tmp = dt*dq*domega ;
	for( int i=0 ; i<4 ; i++ )
		for( int j=0 ; j<3 ; j++ )
			A.at<float>( i+3, j+10 ) = tmp.at<float>(i, j) ;
	memcpy( X_tmp.data, X.data, X_dim*sizeof(float) ) ;
	for( int i=0 ; i<X_dim ; i++ )
		memcpy( sigma_tmp.data+( sigma_tmp.cols*i*sizeof(float) ), sigma.data+( sigma.cols*i*sizeof(float)), 
		        sigma_tmp.cols*sizeof(float) ) ;
	X_tmp = A*X_tmp ;
	sigma_tmp = A*sigma_tmp*A.t()  ;

	memcpy( X.data, X_tmp.data, X_dim*sizeof(float) ) ;
	for( int i=0 ; i<X_dim ; i++ )
		memcpy( sigma.data+( sigma.cols*i*sizeof(float)), 
			sigma_tmp.data+( sigma_tmp.cols*i*sizeof(float) ), sigma_tmp.cols*sizeof(float) ) ;
}

void SLAM::measure(Pts3D &observedPoints, cv::Mat &descrsLeft, cv::Mat &descrsRight, int64_t timestamp_ns)
{
	int f_n = descrsLeft.rows ;
	int idx ;
	vector<int> observeList ; //local
	vector<int> matchList ; //map
	vector<int> addList ;
	/*match*/
	vector<DMatch> matches_left, matches_right;
	matcher->match(descrsLeft, leftMap, matches_left);
	matcher->match(descrsRight, rightMap, matches_right);
	for( int i=0 ; i<y_size ; i++ )
		left_matchList[i] = right_matchList[i] = -1 ;
	for( int i=0 ; i<matches_left.size() ; i++ )
		left_matchList[ matches_left[i].queryIdx ] = matches_left[i].trainIdx ;
	for( int i=0 ; i<matches_right.size() ; i++ )
		right_matchList[ matches_right[i].queryIdx ] = matches_right[i].trainIdx ;
	for( int i=0 ; i<f_n ; i++ )
		if( left_matchList[i] == right_matchList[i] )
		{
			if( left_matchList[i] == -1 )
				addList.push_back( i ) ;
			else
			{
				observeList.push_back( i ) ;
				matchList.push_back( left_matchList[i] ) ;
			}
		}
	/*measure*/
	for( int _idx=0 ; _idx<matchList.size() ; _idx++ )	
	{
		generateR( R_inv, X.at<float>(3,0), X.at<float>(4,0), X.at<float>(5,0), X.at<float>(6,0) ) ;
		R_inv = R_inv.t() ;
		nR_inv = (-1)*R_inv ;
		idx = matchList[ _idx ] ;
		//H
		for( int i=0 ; i<3 ; i++ )
		{
			memcpy( H.data+(i*H.cols*sizeof(float)), nR_inv.data+(i*nR_inv.cols*sizeof(float)), 
			        nR_inv.cols*sizeof(float) ) ;
			memcpy( H.data+( (i*H.cols+X_dim+idx*3 ) *sizeof(float)), 
			        R_inv.data+(i*R_inv.cols*sizeof(float)), 
				R_inv.cols*sizeof(float) ) ;
		}
		//H*sigma*H
		generate_HsigmaH( H_sigma_H, idx, R_inv, nR_inv, sigma ) ;
		generate_sigmaH( sigma_H, idx, sigma, R_inv ) ;
		K = sigma_H*( H_sigma_H+R ).inv() ;
		//generate HX
		generate_HX( HX, idx, R_inv ) ;
		int j = observeList[_idx] ;
		y.at<float>(0,0) = observedPoints[j].x ;
		y.at<float>(1,0) = observedPoints[j].y ;
		y.at<float>(2,0) = observedPoints[j].z ;
		X = X + K*( y-HX ) ;
		sigma = sigma-K*( H*sigma ) ;

		for( int i=0 ; i<3 ; i++ )
			memset( H.data+( (i*H.cols+X_dim+idx*3 ) *sizeof(float) ), 0, R_inv.cols*sizeof(float)  ) ;
	}
	//update map
	for( int i=0 ; i<addList.size() ; i++ )
	{
		idx = addList[i] ;
		memcpy( leftMap.data+y_size*32*sizeof(char), descrsLeft.data+idx*32*sizeof(char), 32*sizeof(char) ) ;
		memcpy( rightMap.data+y_size*32*sizeof(char), descrsRight.data+idx*32*sizeof(char), 32*sizeof(char) ) ;
		y.at<float>(0,0) = observedPoints[idx].x ;
		y.at<float>(1,0) = observedPoints[idx].y ;
		y.at<float>(2,0) = observedPoints[idx].z ;
		
		/**transform**/
		memcpy( r.data, X.data, 3*sizeof(float) ) ;
		y = R_inv.t()*y+r ;
		//
		memcpy( X.data+( X_dim+3*y_size )*sizeof(float), y.data, 3*sizeof(float) ) ;
		y_size++ ;
	}
	previous_t = timestamp_ns ;
}

void SLAM::generate_dqw( Mat &dqw, float _w, float _x, float _y, float _z )
{
	dqw = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dqw.at<float>(0, 0) = _w, dqw.at<float>(0, 1) = -_x, dqw.at<float>(0, 2) = -_y, dqw.at<float>(0, 3) = -_z ;
	dqw.at<float>(1, 0) = _x, dqw.at<float>(1, 1) = _w, dqw.at<float>(1, 2) = _z, dqw.at<float>(1, 3) = -_y ;
	dqw.at<float>(2, 0) = _y, dqw.at<float>(2, 1) = -_z, dqw.at<float>(2, 2) = _w, dqw.at<float>(2, 3) = _x ;
	dqw.at<float>(3, 0) = _z, dqw.at<float>(3, 1) = _y, dqw.at<float>(3, 2) = -_x, dqw.at<float>(3, 3) = _w ;
}

void SLAM::generate_dq( Mat &dq, float w, float x, float y, float z ) 
{
	dq = Mat::zeros( 4, 4, CV_32FC1 ) ;
	dq.at<float>(0, 0) = w, dq.at<float>(0, 1) = -x, dq.at<float>(0, 2) = -y, dq.at<float>(0, 3) = -z ;
	dq.at<float>(1, 0) = x, dq.at<float>(1, 1) = w, dq.at<float>(1, 2) = -z, dq.at<float>(1, 3) = y ;
	dq.at<float>(2, 0) = y, dq.at<float>(2, 1) = z, dq.at<float>(2, 2) = w, dq.at<float>(2, 3) = -x ;
	dq.at<float>(3, 0) = z, dq.at<float>(3, 1) = -y, dq.at<float>(3, 2) = x, dq.at<float>(3, 3) = w ;
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

void SLAM::generateR( cv::Mat &R, float q0, float q1, float q2, float q3 )
{
	R.at<float>(0,0) = 1-2*(q2*q2+q3*q3), R.at<float>(0,1) = 2*(q1*q2-q0*q3), R.at<float>(0,2) = 2*(q0*q2+q1*q3) ;
	R.at<float>(1,0) = 2*(q1*q2+q0*q3), R.at<float>(1,1) = 1-2*(q1*q1+q3*q3), R.at<float>(1,2) = 2*(q2*q3-q0*q1) ;
	R.at<float>(2,0) = 2*(q1*q3-q0*q2), R.at<float>(2,1) = 2*(q0*q1+q2*q3), R.at<float>(2,2) = 1-2*(q1*q1+q2*q2) ;
}

void SLAM::generate_HsigmaH( cv::Mat &H_sigma_H, int idx, cv::Mat &R_inv, cv::Mat &nR_inv, cv::Mat &sigma )
{
	//H_y
	for( int i=0 ; i<3 ; i++ )
	{
		memcpy( H_y.data+( i*H_y.cols*sizeof(float) ), 
			nR_inv.data+( i*nR_inv.cols*sizeof(float) ), nR_inv.cols*sizeof(float)  ) ;
		memcpy( H_y.data+( i*H_y.cols+3 )*sizeof(float), 
			R_inv.data+( i*R_inv.cols )*sizeof(float), R_inv.cols*sizeof(float) ) ;
	}
	//sigma_ry
	for( int i=0 ; i<3 ; i++ )
	{
		memcpy( sigma_ry.data+(i*sigma_ry.cols)*sizeof(float), 
			sigma.data+(i*sigma.cols)*sizeof(float), 3*sizeof(float) ) ;
		memcpy( sigma_ry.data+(i*sigma_ry.cols+ 3 )*sizeof(float), 
			sigma.data+(i*sigma.cols+ X_dim+3*idx )*sizeof(float), 3*sizeof(float) ) ;
	}
	for( int i=0 ; i<3 ; i++ )
	{
		int y_pos = X_dim+3*idx  ;
		memcpy( sigma_ry.data+( (3+i)*sigma_ry.cols )*sizeof(float), sigma.data+( y_pos+i)*sigma.cols *sizeof(float),
			3*sizeof(float) ) ;
		memcpy( sigma_ry.data+( (3+i)*sigma_ry.cols+3 )*sizeof(float),
			sigma.data+( (y_pos+i )*sigma.cols + y_pos  )*sizeof(float), 3*sizeof(float) ) ;
	}
	H_sigma_H = H_y*sigma_ry*( H_y.t() ) ;
}

void SLAM::generate_sigmaH( cv::Mat &sigma_H, int idx, cv::Mat &sigma, cv::Mat &R_inv ) 
{
	for( int i=0 ; i<3 ; i++ )
	{
		memcpy( sigma_r.data+ i*sigma_r.cols*sizeof(float) , sigma.data+i*sigma.cols*sizeof(float), 
		        sigma.cols*sizeof(float) ) ;
		memcpy( sigma_y.data+ i*sigma_y.cols*sizeof(float) , sigma.data+(X_dim+idx*3+i )*sigma.cols*sizeof(float), 
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
