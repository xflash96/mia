#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "slam.h"
#include "cam.h"

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
		0.01f, 0.02f, 0.04f
	};
	float _omega[] = {
		0, 0, 0.0
	};

	v = Mat(3, 1, CV_32FC1, _v).clone();
	omega = Mat(3, 1, CV_32FC1, _omega).clone();
}

const int L = 2;
void square_map(Mat &x, Mat &rvec, Mat &M, Mat &descr)
{
	float _x[] = {
		-3, -3, -3
	};
	x = Mat(3, 1, CV_32FC1, _x).clone();
	float _M[L*3] = {
		0, 0, 0,
		0, 0, 1,
	} ;
	/*
		0, 1, 0,
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

bool run_camera(Mat x, Mat rvec, Mat v, Mat omega, Mat M, Mat descr, float noise=0.)
{
	srand(0);
	float dt = 1.f;
	SLAM slam ;
	for(int i=0; i<1000; i++){
		cerr << "i = " << i << endl;
		x = x + dt*v;
		rvec = rvec + dt*omega;
		cerr << "rvec\n" << rvec << endl ;
		Mat R;
		Rodrigues(rvec, R);
		Scalar xp(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
		cerr << "x " << x << " " << rvec << endl;
		//cerr << "camara R\n" << R << endl ;
		Mat Z = R*(M-xp);
		Pts3D Zp;
		Mat_to_Pts3D(Z, Zp);

		for(int j=0; j<(int)Zp.size(); j++){
			Point3f n;
			n.x = norm_rand(0, noise);
			n.y = norm_rand(0, noise);
			n.z = norm_rand(0, noise);
			Zp[j] += n;
		}
		cerr << "************pos" << endl ;
		cerr << Zp[0] << endl ;
		cerr << Zp[1] << endl ;
		cerr << Zp[2] << endl ;
		cerr << Zp[3] << endl ;

		//TODO add SLAM here
		if( i == 0 )
		{
			slam.initial( Zp, descr, descr, i*dt ) ;
			//cerr << "****X\n" ;
			//cerr << slam.X.t() << endl ;
			//cerr << "*****sigma\n" ;
			//cerr << slam.sigma << endl ;
		}
		else
		{
			slam.predict(i*dt*1e7 );
			//cerr << "****predict X\n" ;
			//cerr << slam.X.t() << endl ;
			slam.measure(Zp, descr, descr, i*dt*1e7 );
			//cerr << "****X\n" ;
			//cerr << slam.X.at<float>(10, 0) << " " << slam.X.at<float>(11, 0) << " "<< slam.X.at<float>(12, 0) << " " << endl ;
			//cerr << "*****sigma\n" ;
			//cerr << slam.sigma << endl ;
		}
		cerr << Zp << endl;
		//cerr << "*******var" << endl ;
		//for( int j=13 ; j<13+24 ; j++ )
		//	cerr << slam.sigma.at<float>( j, j ) << " " ;
		//cerr << endl ;
	}
	for( int i=0 ; i<L ; i++ )
		cerr << slam.X.at<float>( 13+3*i, 0 ) <<  " " << slam.X.at<float>( 13+3*i+1, 0 ) << " " << slam.X.at<float>( 13+3*i+2, 0 ) << endl;

	//for( int j=13 ; j<13+24 ; j++ )
	//	cerr << slam.sigma.at<float>( j, j ) << " " ;
	return true;
}

int main()
{
	Mat x, rvec;
	Mat v, omega;
	Mat M, descr;
	cerr << "rvec\n" << rvec << endl ;
	square_map(x, rvec, M, descr);
	striaght_cnst_speed(v, omega);
	run_camera(x, rvec, v, omega, M, descr, 0.0001);
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
	return 0;
}
