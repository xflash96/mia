#include "gui.h"
#include "mia.h"

static gboolean GUI_expose (GtkWidget *da, GdkEventExpose *event, gpointer data)
{
	return GUI_THR->expose(da, event, data);
}
static gboolean GUI_configure (GtkWidget *da, GdkEventConfigure *event, gpointer data)
{
	return GUI_THR->configure(da, event, data);
}

static gboolean GUI_rotate (gpointer data)
{
	return GUI_THR->rotate(data);
}


GUI::GUI()
{
	init_window();
}

void GUI::init_window()
{
	GtkWidget *window, *glcanvas;
	GdkGLConfig *glconfig;

	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_default_size (GTK_WINDOW(window), 800, 600);
	glcanvas = gtk_drawing_area_new();

	gtk_container_add (GTK_CONTAINER(window), glcanvas);
	gtk_widget_set_events (glcanvas, GDK_EXPOSURE_MASK);

	g_signal_connect_swapped (window, "destroy", 
			G_CALLBACK (end_program), NULL);

	gtk_widget_show (window);

	glconfig = gdk_gl_config_new_by_mode((GdkGLConfigMode)(
			GDK_GL_MODE_RGB |
			GDK_GL_MODE_DEPTH |
			GDK_GL_MODE_DOUBLE));

	if(!glconfig)
		g_assert_not_reached ();
	if(!gtk_widget_set_gl_capability 
		(glcanvas, glconfig, NULL, TRUE, GDK_GL_RGBA_TYPE)){
		g_assert_not_reached ();
	}

	g_signal_connect (glcanvas, "configure-event", 
			G_CALLBACK(GUI_configure), NULL);
	g_signal_connect (glcanvas, "expose-event",
			G_CALLBACK(GUI_expose), NULL);

	gtk_widget_show_all (window);

	g_timeout_add (1000/60, GUI_rotate, glcanvas);
}

float boxv[][3] = {
	{ -0.5, -0.5, -0.5 },
	{  0.5, -0.5, -0.5 },
	{  0.5,  0.5, -0.5 },
	{ -0.5,  0.5, -0.5 },
	{ -0.5, -0.5,  0.5 },
	{  0.5, -0.5,  0.5 },
	{  0.5,  0.5,  0.5 },
	{ -0.5,  0.5,  0.5 }
};
#define ALPHA 0.5

static float ang = 30.;
gboolean GUI::expose (GtkWidget *da, GdkEventExpose *event, gpointer data)
{
	GdkGLContext *glcontext = gtk_widget_get_gl_context (da);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (da);
	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
		g_assert_not_reached();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	glRotatef (ang, 1, 0, 1);
	// glRotatef (ang, 0, 1, 0);
	// glRotatef (ang, 0, 0, 1);

	glShadeModel(GL_FLAT);

#if 1
	glBegin (GL_QUADS);
	glColor4f(0.0, 0.0, 1.0, ALPHA);
	glVertex3fv(boxv[0]);
	glVertex3fv(boxv[1]);
	glVertex3fv(boxv[2]);
	glVertex3fv(boxv[3]);

	glColor4f(1.0, 1.0, 0.0, ALPHA);
	glVertex3fv(boxv[0]);
	glVertex3fv(boxv[4]);
	glVertex3fv(boxv[5]);
	glVertex3fv(boxv[1]);
	
	glColor4f(0.0, 1.0, 1.0, ALPHA);
	glVertex3fv(boxv[2]);
	glVertex3fv(boxv[6]);
	glVertex3fv(boxv[7]);
	glVertex3fv(boxv[3]);
	
	glColor4f(1.0, 0.0, 0.0, ALPHA);
	glVertex3fv(boxv[4]);
	glVertex3fv(boxv[5]);
	glVertex3fv(boxv[6]);
	glVertex3fv(boxv[7]);
	
	glColor4f(1.0, 0.0, 1.0, ALPHA);
	glVertex3fv(boxv[0]);
	glVertex3fv(boxv[3]);
	glVertex3fv(boxv[7]);
	glVertex3fv(boxv[4]);
	
	glColor4f(0.0, 1.0, 0.0, ALPHA);
	glVertex3fv(boxv[1]);
	glVertex3fv(boxv[5]);
	glVertex3fv(boxv[6]);
	glVertex3fv(boxv[2]);

	glEnd ();
#endif

	glBegin (GL_LINES);
	glColor3f (1., 0., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (1., 0., 0.);
	glEnd ();
	
	glBegin (GL_LINES);
	glColor3f (0., 1., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 1., 0.);
	glEnd ();
	
	glBegin (GL_LINES);
	glColor3f (0., 0., 1.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 0., 1.);
	glEnd ();

	glBegin(GL_LINES);
	glColor3f (1., 1., 1.);
	glVertex3fv(boxv[0]);
	glVertex3fv(boxv[1]);
	
	glVertex3fv(boxv[1]);
	glVertex3fv(boxv[2]);
	
	glVertex3fv(boxv[2]);
	glVertex3fv(boxv[3]);
	
	glVertex3fv(boxv[3]);
	glVertex3fv(boxv[0]);
	
	glVertex3fv(boxv[4]);
	glVertex3fv(boxv[5]);
	
	glVertex3fv(boxv[5]);
	glVertex3fv(boxv[6]);
	
	glVertex3fv(boxv[6]);
	glVertex3fv(boxv[7]);
	
	glVertex3fv(boxv[7]);
	glVertex3fv(boxv[4]);
	
	glVertex3fv(boxv[0]);
	glVertex3fv(boxv[4]);
	
	glVertex3fv(boxv[1]);
	glVertex3fv(boxv[5]);
	
	glVertex3fv(boxv[2]);
	glVertex3fv(boxv[6]);
	
	glVertex3fv(boxv[3]);
	glVertex3fv(boxv[7]);
	glEnd();

	glPopMatrix ();

	if (gdk_gl_drawable_is_double_buffered (gldrawable))
		gdk_gl_drawable_swap_buffers (gldrawable);

	else
		glFlush ();

	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

gboolean
GUI::configure (GtkWidget *da, GdkEventConfigure *event, gpointer user_data)
{
	GdkGLContext *glcontext = gtk_widget_get_gl_context (da);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (da);

	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
	{
		g_assert_not_reached ();
	}

	glLoadIdentity();
	glViewport (0, 0, da->allocation.width, da->allocation.height);
	glOrtho (-10,10,-10,10,-20050,10000);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glScalef (10., 10., 10.);
	
	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

gboolean
GUI::rotate (gpointer user_data)
{
	GtkWidget *da = GTK_WIDGET (user_data);

	ang++;

	gdk_window_invalidate_rect (da->window, &da->allocation, FALSE);
	gdk_window_process_updates (da->window, FALSE);

	return TRUE;
}
