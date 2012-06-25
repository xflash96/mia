#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "async.h"

void main_loop_add_fd(int fd, GIOFunc callback, gpointer data, gint priority)
{
	GIOChannel *chan = NULL;
	chan = g_io_channel_unix_new(fd);
	g_io_add_watch_full(chan, priority, G_IO_IN, callback, data, NULL);
}

AsyncQueue::AsyncQueue(GIOFunc on_notify)
{
	if (0 > pipe(msg_pipe)) {
		fprintf(stderr, "PIPE\n");
		exit(-1);
	}
	queue = g_async_queue_new();
	main_loop_add_fd (msg_pipe[0], on_notify, NULL, G_PRIORITY_HIGH);
}

AsyncQueue::~AsyncQueue()
{
	g_async_queue_unref(queue);
}

void AsyncQueue::push(gpointer data)
{
	g_async_queue_push(queue, data);
}

gpointer AsyncQueue::pop()
{
	return g_async_queue_pop(queue);
}

void AsyncQueue::notify()
{
	write(msg_pipe[1], "1", 1);
}
