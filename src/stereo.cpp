#include "stereo.h"
#include "mia.h"

using namespace cv;

int ctr = 0;


Stereo::Stereo()
{
	int n_feature = 20;
	left_orb = 	new ORB(n_feature, 1.2f, 1, 31, 0, 2, ORB::FAST_SCORE, 31);
	right_orb = 	new ORB(n_feature, 1.2f, 1, 31, 0, 2, ORB::FAST_SCORE, 31);
	matcher = 	new BFMatcher(NORM_HAMMING);
}
Stereo::~Stereo()
{
	if(left_orb)
		delete left_orb;
	if(right_orb)
		delete right_orb;
	if(matcher)
		delete matcher;
}

void matches_to_Pts2D(vector<DMatch> &matches, vector<KeyPoint> &left_keys, vector<KeyPoint> &right_keys, Pts2D &left_pts, Pts2D &right_pts)
{
	left_pts.clear();
	right_pts.clear();
	for(size_t i=0; i<matches.size(); i++){
		int li = matches[i].queryIdx;
		int ri = matches[i].trainIdx;
		left_pts.push_back(left_keys[li].pt);
		right_pts.push_back(right_keys[ri].pt);
	}
}

bool Stereo::get_feat_pts
(Mat &left_img, Mat &right_img, Pts3D &feat_pts, Mat &left_descrs, Mat &right_descrs)
{
	feat_pts.clear();
	vector<KeyPoint> left_keys, right_keys;

#if 1
	(*left_orb)(left_img, Mat(), left_keys, left_descrs, false);
	(*right_orb)(right_img, Mat(), right_keys, right_descrs, false);
#else
	Mat new_left, new_right;
	cvtColor(left_img, new_left, CV_RGB2GRAY);
	cvtColor(right_img, new_right, CV_RGB2GRAY);
	FAST(new_left, left_keys, 31, true);
	FAST(new_right, right_keys, 31, true);
#endif

	vector<DMatch> matches;
	matcher->match(left_descrs, right_descrs, matches);

	Pts2D left_pts, right_pts;
	matches_to_Pts2D(matches, left_keys, right_keys, left_pts, right_pts);

	Pts2D left_udpts, right_udpts;
	if(left_pts.empty())
		return false;
	left.undistortPoints(left_pts, left_udpts);
	right.undistortPoints(right_pts, right_udpts);

	Mat lh = Mat(3, 1, CV_64FC1);
	Mat rh = lh.clone();
	Mat err;

	Pts2D left_mpts, right_mpts;
	vector<DMatch> mmatches;
	double err_threshold = 0.3;

	for(size_t i=0; i<left_udpts.size(); i++){
		Point2f lp = left_udpts[i];
		Point2f rp = right_udpts[i];

		lh.at<double>(0,0) = lp.x;
		lh.at<double>(1,0) = lp.y;
		lh.at<double>(2,0) = 1;

		rh.at<double>(0,0) = rp.x;
		rh.at<double>(1,0) = rp.y;
		rh.at<double>(2,0) = 1;
		lh = left.cam_mat*lh;
		rh = right.cam_mat*rh;

		//cerr << lh << rh << endl;
		err = rh.t()*F*lh;
		double err_d = abs(err.at<double>(0,0));
		if(err_d > err_threshold)
			continue;

		//cerr << err_d << endl;

		left_mpts.push_back(left_udpts[i]);
		right_mpts.push_back(right_udpts[i]);
		mmatches.push_back(matches[i]);
	}

	if(left_mpts.size() <= 3)
		return false;

//	correctMatches(F, left_mpts, right_mpts, left_mpts, right_mpts);

	Pts3D pts_3d;
	triangulatePoints(left_mpts, right_mpts, feat_pts);

	Mat left_mdes = Mat(left_mpts.size(), left_descrs.cols, CV_8UC1);
	Mat right_mdes = left_mdes.clone();

	for(size_t i=0; i<left_mpts.size(); i++){
		int lidx = mmatches[i].queryIdx;
		int ridx = mmatches[i].trainIdx;
		left_mdes.row(i) = left_descrs.row(lidx);
		right_mdes.row(i) = right_descrs.row(ridx);
	}
	left_descrs = left_mdes;
	right_descrs = right_mdes;

//	cerr << feat_pts << endl << endl;


//	drawKeypoints(left_img, left_keys, left_img);
//	drawKeypoints(right_img, right_keys, right_img);

	Mat match_img;
	drawMatches(left_img, left_keys, right_img, right_keys, mmatches, match_img);
	imshow("match", match_img);
	//cerr << feat_pts << endl;

	Pts2D feat_reproj;
	HD_THR->hd_cam.projectPoints(feat_pts, feat_reproj);
	if(HD_THR->hd_img.empty())
		return true;
	Mat reproj_canvas = HD_THR->hd_img.clone();
	for(size_t i=0; i<feat_reproj.size(); i++){
		Point2f pt = feat_reproj[i];
		circle(reproj_canvas, pt, 5, Scalar(255, 0, 0), 3);
	}
	cerr << feat_reproj << endl;
	imshow("reproj", reproj_canvas);
//	imshow("left_feat", left_img);
//	imshow("right_feat", right_img);
	return true;
}

void Stereo::triangulatePoints(Pts2D left_pts, Pts2D right_pts, Pts3D &dst_pts)
{
	Mat pts_4d;
	Mat left_mat, right_mat;

	left_mat = Mat(left_pts);
	right_mat = Mat(right_pts);
	left_mat = left_mat.reshape(1, left_pts.size()).t();
	right_mat= right_mat.reshape(1, right_pts.size()).t();

	cv::triangulatePoints(left.proj, right.proj, left_mat, right_mat, pts_4d);
	cv::convertPointsFromHomogeneous(pts_4d.t(), dst_pts);
}

void Stereo::load_F(FileStorage &fs)
{
	fs["F"] >> F;
}
