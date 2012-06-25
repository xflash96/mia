#ifndef __MIA_H__
#define __MIA_H__

#include <stdint.h>
#ifndef INT64_MAX
# define INT64_MAX 0x7fffffffffffffffLL // more reasonable than use __STDC
# endif

#include "decoder.h"
#include "convert.h"
#include "calib.h"
#include "iothread.h"
#include "gui.h"

extern int64_t process_start_time;

extern StereoThread *STEREO_THR;
extern HDVideoThread *HD_THR;
extern GUI *GUI_THR;

void end_program(void);

/*
class MiaContext
{
public:
	enum State
	{
		CALIB_STEREO,
		CALIB_HDVIDEO,
		ONLINE,
		REPLAY
	}state;

	bool record_video;

} *CTX;
*/

#endif
