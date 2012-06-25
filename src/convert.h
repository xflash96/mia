#ifndef __CONVERT_H__
#define __CONVERT_H__

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}
#include <opencv2/opencv.hpp>
#include <stdarg.h>

void errno_exit(const char *tmpl, ...);
cv::Mat raw_to_cvmat(uint8_t *data, int width, int height, enum PixelFormat format);
cv::Mat avframe_to_cvmat(AVFrame *frame);
int64_t timespec_to_ns(struct timespec *t);
int64_t timeval_to_ns(struct timeval *t);
int64_t time_now_ns();

#endif
