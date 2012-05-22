extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}
#include <opencv2/opencv.hpp>

cv::Mat avframe_to_cvmat(AVFrame *frame);
