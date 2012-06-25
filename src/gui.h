#ifndef __GUITHREAD_H__
#define __GUITHREAD_H__

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include "cam.h"
#include "async.h"

struct GUIPacket{
	Pts3D pts, scales;
};
class GUI{
public:
	GUI();
	void init_window();
	void redraw();

	AsyncQueue *queue;
	Pts3D map_pts, map_scales;
	GtkWidget *glcanvas;

	gboolean on_msg
		(GIOChannel *source, GIOCondition condition, gpointer data);
	gboolean configure 
		(GtkWidget *da, GdkEventConfigure *event, gpointer user_data);
	gboolean expose 
		(GtkWidget *da, GdkEventExpose *event, gpointer user_data);
};

#endif
