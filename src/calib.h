#include <opencv2/opencv.hpp>
typedef struct{
	cv::Mat cam_mat, distort; //intrinsics
	cv::Mat extr; //extrinsics
}CamParams;
bool calib_stereo_cameras(cv::Mat left_img, cv::Mat right_img, const char *intrinsic_path, CamParams &left_camp, CamParams &right_camp);
bool calib_hd_camera(cv::Mat img, const char *cam_params_path, CamParams &camp);
