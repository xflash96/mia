#include <stdio.h>
#include <unistd.h>
#include "calib.h"

using namespace std;
using namespace cv;

inline float NORM(Point3f p, Point3f q)
{
	float a = p.x-q.x, b = p.y-q.y, c = p.z-q.z;
	return (float)sqrt(a*a+b*b+c*c);
}
static float chessboard_squaresize = 2.91f;
bool find_and_draw_chessboard(Size pattern, Mat &img, vector<Point2f> &corners, Mat &canvas)
{
	Mat gray;
	cvtColor(img, gray, CV_RGB2GRAY);
	bool pattern_found = 
		findChessboardCorners(gray, pattern, corners, 
			CALIB_CB_ADAPTIVE_THRESH
			| CALIB_CB_NORMALIZE_IMAGE
			| CALIB_CB_FAST_CHECK );

	if( pattern_found )
		cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), 
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,
				30, 0.1) );
	drawChessboardCorners(canvas, pattern, Mat(corners), pattern_found);
	return pattern_found;
}

void build_chessbord_grid(Size pattern, float square_size, vector<Point3f> &grid)
{
	for(int i=0; i<pattern.height; i++)
		for(int j=0; j<pattern.width; j++)
			grid.push_back(Point3f(j*square_size, i*square_size, 0.f));
}

bool calib_stereo_cameras(Mat left_img, Mat right_img, Cam left_cam, Cam right_cam)
{
	Size chessboard_pattern(8,6);
	Mat left_canvas(left_img), right_canvas(right_img);
	vector<Point2f> left_corners, right_corners;

	static vector<vector<Point2f> > left_grids, right_grids;
	static vector<vector<Point3f> > grid_coords;
	static vector<Point3f> grid;
	if( grid.empty() )
		build_chessbord_grid(chessboard_pattern, chessboard_squaresize, grid);


	bool lfound = find_and_draw_chessboard( chessboard_pattern,
			left_img, left_corners, left_canvas);
	bool rfound =find_and_draw_chessboard( chessboard_pattern,
			right_img, right_corners, right_canvas);

	if( !(lfound && rfound) )
		return false;

	imshow("left_calib", left_canvas);
	imshow("right_calib", right_canvas);
	moveWindow("left_calib", 0, left_img.rows+60);
	moveWindow("right_calib", left_img.cols, left_img.rows+60);

	int r = waitKey(0);
	if ((char)r == 'x')
		return false;
	fprintf(stderr, "key = %c\n", (char)r);
	grid_coords.push_back(grid);

	left_grids.push_back(left_corners);
	right_grids.push_back(right_corners);

	Mat R, T, E, F;
	double rms = stereoCalibrate(grid_coords, left_grids, right_grids,
		left_cam.cam_mat, left_cam.dist_coeff,
		right_cam.cam_mat, right_cam.dist_coeff,
		left_img.size(), R, T, E, F,
		TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01),
		CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 |
		CV_CALIB_SAME_FOCAL_LENGTH |
		CV_CALIB_ZERO_TANGENT_DIST);
	cerr << left_cam.cam_mat << left_cam.dist_coeff
		<< right_cam.cam_mat << right_cam.dist_coeff
		<< endl;
	bool ok = checkRange(left_cam.cam_mat) 
		&& checkRange(right_cam.cam_mat)
		&& checkRange(left_cam.dist_coeff) 
		&& checkRange(right_cam.dist_coeff)
		&& grid_coords.size() > 16;
	fprintf(stderr, "%lu\tcalibration RMS = %.3lf, %d\n", 
			grid_coords.size(),rms, ok);

	// construct projection matrix
	Mat StereoProj = Mat(3, 4, CV_64FC1);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			StereoProj.at<double>(i,j) = R.at<double>(i,j);

	for(int i=0; i<3; i++)
		StereoProj.at<double>(i,3) = T.at<double>(i,0);

	double _IDProj[] = {	1., 0., 0., 0.,
				0., 1., 0., 0.,
				0., 0., 1., 0.};
	Mat IDProj = Mat(3, 4, CV_64FC1, _IDProj);

	left_cam.proj = IDProj;
	right_cam.proj = StereoProj;
	left_cam.rvec = Mat(3, 1, CV_32FC1, Scalar(0));
	left_cam.tvec = Mat(3, 1, CV_32FC1, Scalar(0));
	Rodrigues(R, right_cam.rvec);
	right_cam.tvec = T;

	return ok;
}

bool calib_hd_camera(Mat img, const char *cam_params_path, Cam &cam)
{
	Size chessboard_pattern(8,6);
	Mat canvas(img);
	vector<Point2f> corners;

	static vector<vector<Point2f> > hd_grids;
	static vector<vector<Point3f> > grid_coords;
	static vector<Point3f> grid;
	if( grid.empty() )
		build_chessbord_grid(chessboard_pattern, chessboard_squaresize, grid);


	bool found = find_and_draw_chessboard( chessboard_pattern,
			img, corners, canvas);

	if( !found )
		return false;

	imshow("hd_calib", canvas);
	int r = waitKey(0);
	if ((char)r == 'x')
		return false;
	grid_coords.push_back(grid);

	hd_grids.push_back(corners);

	vector<Mat> rvec, tvec;
	double rms = calibrateCamera(grid_coords, hd_grids,
			img.size(),
			cam.cam_mat, cam.dist_coeff,
			rvec, tvec,
			CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5,
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01)
			);
	cerr << cam.cam_mat << cam.dist_coeff<< endl;
	bool ok = checkRange(cam.cam_mat) 
		&& checkRange(cam.dist_coeff) 
		&& grid_coords.size() > 16;
	fprintf(stderr, "%lu\tcalibration RMS = %.3lf, %d\n", 
			grid_coords.size(),rms, ok);

	return ok;
}

double chessboard_neighbor_error(Pts3D &chessboard_3d, Size &chessboard_pattern)
{
	double sum_err = 0;
	int w = chessboard_pattern.width;
	int h = chessboard_pattern.height;
	for(int i=0; i<h; i++)
		for(int j=0; j<w; j++){
			Point3f p = chessboard_3d[i*w+j];
			double sq=0, sr=0;
			if(i!=0){
				Point3f q = chessboard_3d[(i-1)*w+j];
				sq = NORM(p, q)-chessboard_squaresize;
				sq *= sq;
				fprintf(stderr, "%d %d = %lf\n", i, j, sq);
			}
			if(j!=0){
				Point3f r = chessboard_3d[i*w+j-1];
				sr = NORM(p, r)-chessboard_squaresize;
				sr *= sr;
				fprintf(stderr, "%d %d = %lf -\n", i, j, sr);
			}
			sum_err += sr+sq;
		}
	sum_err /= w*h;
	return sum_err;
}

// find all pair of projection mat
bool calib_cameras_poses(Mat left_img, Mat right_img, Mat hd_img,
		Stereo *stereo, Cam *hd_cam)
{
	if(left_img.empty() || right_img.empty() || hd_img.empty())
		return false;
	hd_img = hd_img.clone();

	// build the 4D chessboard coordinate
	Size chessboard_pattern(8,6);
	Mat left_canvas(left_img), right_canvas(right_img), hd_canvas = hd_img.clone();
	vector<Point2f> left_corners, right_corners, hd_corners;

	vector<Point3f> grid;
	build_chessbord_grid(chessboard_pattern, chessboard_squaresize, grid);

	bool lfound = find_and_draw_chessboard( chessboard_pattern,
			left_img, left_corners, left_canvas);
	bool rfound =find_and_draw_chessboard( chessboard_pattern,
			right_img, right_corners, right_canvas);
	bool hdfound =find_and_draw_chessboard( chessboard_pattern,
			hd_img, hd_corners, hd_canvas);

	Pts3D chessboard_3d;
	if (!(lfound && rfound && hdfound))
		return false;

	imshow("left_calib", left_canvas);
	imshow("right_calib", right_canvas);
	imshow("hd_calib", hd_canvas);
#if 0
	int r = waitKey(0);
	if ((char)r == 'x')
		return false;
#endif

	Pts2D left_udcorners, right_udcorners;

	stereo->left.undistortPoints(left_corners, left_udcorners);
	stereo->right.undistortPoints(right_corners, right_udcorners);

	stereo->triangulatePoints(left_udcorners, right_udcorners, chessboard_3d);
//	cerr << chessboard_3d << endl;

	double err = chessboard_neighbor_error(chessboard_3d, chessboard_pattern);
	fprintf(stderr, "total err = %lf\n", err);
	
	bool ok = hd_cam->solvePnP(chessboard_3d, hd_corners);
	Pts2D reproj_chessboard;
	hd_cam->projectPoints(chessboard_3d, reproj_chessboard);
	Mat reproj_canvas(hd_img);
	for(size_t i=0; i<reproj_chessboard.size(); i++){
		Point2f pt = reproj_chessboard[i];
		circle(reproj_canvas, pt, 5, Scalar(255, 0, 0), 3);
	}
	imshow("reproj", reproj_canvas);
	fprintf(stderr, "ok = %d\n", ok);
	return ok;
}
