extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}
#include <opencv2/opencv.hpp>

cv::Mat raw_to_cvmat(uint8_t *data, int width, int height, enum PixelFormat format);
cv::Mat avframe_to_cvmat(AVFrame *frame);
