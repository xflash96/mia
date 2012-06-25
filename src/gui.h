#ifndef __GUITHREAD_H__
#define __GUITHREAD_H__

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/gl.h>

class GUI{
public:
	GUI();
	void init_window();

	gboolean configure 
		(GtkWidget *da, GdkEventConfigure *event, gpointer user_data);
	gboolean expose 
		(GtkWidget *da, GdkEventExpose *event, gpointer user_data);
	gboolean rotate 
		(gpointer user_data);
};

#endif
