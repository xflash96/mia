#include <opencv2/opencv.hpp>
extern "C"{
#include <stdint.h>
}

void process_stereo(cv::Mat left_img, cv::Mat right_img, int64_t cap_time);