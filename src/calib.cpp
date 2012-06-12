#include <stdio.h>
#include <unistd.h>
#include "calib.h"

using namespace std;
using namespace cv;

static float chessboard_squaresize = 2.93f;
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

bool calib_stereo_cameras(Mat left_img, Mat right_img, const char *cam_params_path,
		CamParams &left_camp, CamParams &right_camp)
{
	if(-1!=access(cam_params_path, F_OK)){
		FileStorage fs(cam_params_path, FileStorage::READ);
		fs["left_cam_mat"] >> left_camp.cam_mat;
		fs["left_distort"] >> left_camp.distort;
		fs["right_cam_mat"] >> right_camp.cam_mat;
		fs["right_distort"] >> right_camp.distort;
		fs.release();
		return true;
	}
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

	Mat R, T, E, F;

	if( lfound && rfound ){
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

		double rms = stereoCalibrate(grid_coords, left_grids, right_grids,
			left_camp.cam_mat, left_camp.distort,
			right_camp.cam_mat, right_camp.distort,
			left_img.size(), R, T, E, F,
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01),
			CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
		cerr << left_camp.cam_mat << left_camp.distort 
			<< right_camp.cam_mat << right_camp.distort
			<< endl;
		bool ok = checkRange(left_camp.cam_mat) 
			&& checkRange(right_camp.cam_mat)
			&& checkRange(left_camp.distort) 
			&& checkRange(right_camp.distort)
			&& grid_coords.size() > 16;
		fprintf(stderr, "%lu\tcalibration RMS = %.3lf, %d\n", 
				grid_coords.size(),rms, ok);

		if(ok){
			FileStorage fs(cam_params_path, FileStorage::WRITE);
			fs << "left_cam_mat" << left_camp.cam_mat;
			fs << "left_distort" << left_camp.distort;
			fs << "right_cam_mat" << right_camp.cam_mat;
			fs << "right_distort" << right_camp.distort;
			fs << "R" << R;
			fs << "T" << T;
			fs << "E" << E;
			fs << "F" << F;
			fs.release();
		}

		return ok;
	}
	return false;
}

bool calib_hd_camera(Mat img, const char *cam_params_path, CamParams &camp)
{
	if(-1!=access(cam_params_path, F_OK)){
		FileStorage fs(cam_params_path, FileStorage::READ);
		fs["hd_cam_mat"] >> camp.cam_mat;
		fs["hd_distort"] >> camp.distort;
		fs.release();
		return true;
	}
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

	if( found ){
		imshow("hd_calib", canvas);
		int r = waitKey(0);
		if ((char)r == 'x')
			return false;
		fprintf(stderr, "key = %c\n", (char)r);
		grid_coords.push_back(grid);

		hd_grids.push_back(corners);

		vector<Mat> rvec, tvec;
		double rms = calibrateCamera(grid_coords, hd_grids,
			img.size(),
			camp.cam_mat, camp.distort,
			rvec, tvec,
			CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5,
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01)
			);
		cerr << camp.cam_mat << camp.distort << endl;
		bool ok = checkRange(camp.cam_mat) 
			&& checkRange(camp.distort) 
			&& grid_coords.size() > 16;
		fprintf(stderr, "%lu\tcalibration RMS = %.3lf, %d\n", 
				grid_coords.size(),rms, ok);

		if(ok){
			FileStorage fs(cam_params_path, FileStorage::APPEND);
			fs << "hd_cam_mat" << camp.cam_mat;
			fs << "hd_distort" << camp.distort;
			fs.release();
		}

		return ok;
	}
	return false;
}

bool calib_cameras(const char* cam_params_path)
{
	if(-1!=access(cam_params_path, F_OK)){
		FileStorage fs(cam_params_path, FileStorage::READ);
		fs.release();
		return true;
	}
	bool ok = false;
	if(ok){
		FileStorage fs(cam_params_path, FileStorage::WRITE);
		fs.release();
	}
	return ok;
}
