#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glib.h>
#include "test.h"
#include "gui.h"
#include "mia.h"

using namespace cv;
using namespace std;

float frand(float min, float max)
{
	return min+(float)((max-min)*rand()*1./RAND_MAX);
}

float norm_rand(float u, float cov)
{
	float x = frand(0, 1);
	return (float)(u+cov*sqrt(-2*log(x))*cos(2*M_PI*x));
}

void striaght_cnst_speed(Mat &v, Mat &omega)
{
	float _v[] = {
		0.01f, 0.02f, 0.01f
	};
	float _omega[] = {
		0, 0, 0.0
	};

	v = Mat(3, 1, CV_32FC1, _v).clone();
	omega = Mat(3, 1, CV_32FC1, _omega).clone();
}

const int L = 3;
void square_map(Mat &x, Mat &rvec, Mat &M, Mat &descr)
{
	float _x[] = {
		-3, -3, -3
	};
	x = Mat(3, 1, CV_32FC1, _x).clone();
	float _M[L*3] = {
		0, 0, 0,
		0, 0, 1,
		0, 1, 0,
	} ;
	/*
		1, 0, 0,
		0, 0.01, 0.01,
		0.01, 0, 0.01,
		0.01, 0.01, 0,
		0.01, 0.01, 0.01,
	};
	*/
	M = Mat(L, 3, CV_32FC1, _M).clone();
	M = M.t();
	descr = Mat(32, L, CV_8UC1);
	for(int i=0; i<L; i++){
		for(int j=0; j<32; j++)
			descr.at<uchar>(j, i) = (uchar)(1<<i);
	}
	descr = descr.t();
}

void SLAMTest::run_camera_once()
{
		x = x + dt*v;
		rvec = rvec + dt*omega;
		Mat R;
		Rodrigues(rvec, R);
		Scalar xp(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
		Mat tmp = Mat::zeros(L,3, CV_32FC1) ;
		for( int i=0 ; i<L ; i++ )
		{
			tmp.at<float>(i,0) = x.at<float>(0, 0) ;
			tmp.at<float>(i,1) = x.at<float>(1, 0) ;
			tmp.at<float>(i,2) = x.at<float>(2, 0) ;
		}
		Mat Z = R*(M-tmp);
		Pts3D Zp;
		Mat_to_Pts3D(Z, Zp);

		for(int j=0; j<(int)Zp.size(); j++){
			Point3f n;
			n.x = norm_rand(0, noise);
			n.y = norm_rand(0, noise);
			n.z = norm_rand(0, noise);
			//Zp[j] += n;
		}

		//TODO add SLAM here
		if( iter == 0 )
		{
			slam.initial( Zp, descr, descr, iter*dt ) ;
		}
		else
		{
			slam.predict(iter*dt*1e7 );
			slam.measure(Zp, descr, descr, iter*dt*1e7 );
		//	cerr << slam.sigma << endl ;
		//	cerr << slam.X << endl ;
		}
		iter++ ;
}

gboolean SLAMTest_run_once(gpointer data)
{
	SLAM_THR->run_camera_once();
	
	Pts3D feat_pts, scales;
	SLAM_THR->slam.feature(feat_pts, scales);
	struct GUIPacket *guip = new GUIPacket();
	guip->pts = feat_pts;
	cerr << feat_pts << scales << endl;
	guip->scales = scales;
	GUI_THR->queue->push(guip);

	return TRUE;
}

SLAMTest::SLAMTest()
{
	square_map(x, rvec, M, descr);
	striaght_cnst_speed(v, omega);
	dt = 1.f;
	noise = 0;
	iter = 0;
	srand(0);

	SLAM_THR = this;

	SLAM_THR->run_camera_once();
	g_timeout_add_full(G_PRIORITY_HIGH, 10, SLAMTest_run_once, NULL, NULL);
//	run_camera(x, rvec, v, omega, M, descr, 0.0001);
	/*
	SLAM s ;
	Mat r = Mat::zeros(3,3,CV_32FC1) ;
	Mat tmp = Mat::eye(3,3,CV_32FC1) ;
	Mat R = Mat::zeros(4,4,CV_32FC1) ;
	Mat _v = Mat::zeros(4,1,CV_32FC1) ;
	float _w, _x, _y, _z ;
	_w = sqrt( (2+sqrt(2)) )/2 ;
	_x = sqrt( (2-sqrt(2)) )/2 ;
	_y = 0 ;
	_z = 0 ;
	_v.at<float>(0,0) = _w;
	_v.at<float>(1,0) = _x;
	_v.at<float>(2,0) = _y;
	_v.at<float>(3,0) = _z;
	s.generate_dq( R, _w, _x, _y, _z  ) ;
	_v = R*_v ;
	float ccc, sss, scc, css, csc, scs, ccs, ssc ;
	float theta[3] ;
	theta[0] = 3.14159/8 ;
	theta[1] = 0 ;
	theta[2] = 0 ;
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
	cerr << "neonbo\n" ;
	cerr << _w << " " << _x << " " << _y << " "<< _z << endl ;

	//s.generateR( r, _x, sqrt( (2-sqrt(2)) )/2, 0, 0  ) ;
	s.generateR( r, _v.at<float>(0,0), _v.at<float>(1,0), _v.at<float>(2,0), _v.at<float>(3,0)  ) ;
	cerr << r << endl ;
	for( int i=0 ; i<2 ; i++ )
		tmp = tmp*r ;
	cerr << tmp << endl ;
	cerr << "test\n" ;
	*/
}
