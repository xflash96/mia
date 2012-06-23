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
		0.1f, 0.1f, 0.12f
	};
	float _omega[] = {
		0, 0, 0
	};

	v = Mat(3, 1, CV_32FC1, _v).clone();
	omega = Mat(3, 1, CV_32FC1, _omega).clone();
}

void square_map(Mat &x, Mat &rvec, Mat &M, Mat &descr)
{
	float _x[] = {
		-3.f, -3.f, -3.f
	};
	x = Mat(3, 1, CV_32FC1, _x).clone();
	const int L = 8;
	float _M[L*3] = {
		0, 0, 0,
		0, 0, 1,
		0, 1, 0,
		1, 0, 0,
		0, 1, 1,
		1, 0, 1,
		1, 1, 0,
		1, 1, 1,
	};
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
	for(int i=0; i<10; i++){
		x = x + dt*i*v;
		rvec = rvec + dt*i*omega;
		Mat R;
		Rodrigues(rvec, R);
		Scalar xp(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
		cerr << "x " << x << " " << rvec << endl;
		Mat Z = R*(M-xp);
		Pts3D Zp;
		Mat_to_Pts3D(Z, Zp);

		for(int j=0; j<(int)Zp.size(); j++){
			Point3f n;
			n.x = norm_rand(0, noise);
			n.y = norm_rand(0, noise);
			n.z = norm_rand(0, noise*j);
			Zp[j] += n;
		}

		//TODO add SLAM here
		if( i == 0 )
			slam.initial( Zp, descr, descr, i*dt ) ;
		else
		{
			slam.predict(i*dt);
			slam.measure(Zp, descr, descr, i*dt);
		}
		cerr << Zp << endl;
	}
	for( int i=0 ; i<8 ; i++ )
		cerr << slam.X.at<float>( 13+3*i, 0 ) <<  " " << slam.X.at<float>( 13+3*i+1, 0 ) << " " << slam.X.at<float>( 13+3*i+2, 0 ) << endl;

	for( int i=0 ; i<24 ; i++ )
	{
		for( int j=0 ; j<24 ; j++ )
			cerr << slam.sigma.at<float>( 13+i, 13+j ) << " " ;
		cerr << endl ;
	}
	return true;
}

int main()
{
	Mat x, rvec;
	Mat v, omega;
	Mat M, descr;
	square_map(x, rvec, M, descr);
	striaght_cnst_speed(v, omega);
	run_camera(x, rvec, v, omega, M, descr, 0.04);

	return 0;
}
