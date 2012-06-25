#ifndef __ASYNC_H__
#define __ASYNC_H__

#include <glib.h>

class AsyncQueue
{
public:
	AsyncQueue(GIOFunc on_notify);
	~AsyncQueue();
	void push(gpointer data);
	gpointer pop();
	void notify();

	int msg_pipe[2];
	GAsyncQueue *queue;
};
void main_loop_add_fd(int fd, GIOFunc callback, gpointer data, gint priority);

#endif
