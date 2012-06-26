#include "gui.h"
#include "mia.h"

using namespace cv;

static gboolean GUI_redraw (gpointer data)
{
	GUI_THR->redraw();
	return TRUE;
}

static gboolean GUI_expose (GtkWidget *da, GdkEventExpose *event, gpointer data)
{
	return GUI_THR->expose(da, event, data);
}
static gboolean GUI_configure (GtkWidget *da, GdkEventConfigure *event, gpointer data)
{
	return GUI_THR->configure(da, event, data);
}

gboolean
GUI_on_msg_wrapper(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return GUI_THR->on_msg(source, condition, data);
}
gboolean
GUI::on_msg(GIOChannel *source, GIOCondition condition, gpointer data)
{
	return TRUE;
}
GUI::GUI()
{
	queue = new AsyncQueue(GUI_on_msg_wrapper);
	init_window();
}

using namespace cv;
void GUI::init_window()
{
	GdkGLConfig *glconfig;

	GtkWidget *window, *glcanvas;
	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	gtk_window_set_default_size (GTK_WINDOW(window), 800, 600);
	glcanvas = gtk_drawing_area_new();
	this->glcanvas = glcanvas;

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
	g_timeout_add_full(G_PRIORITY_HIGH, 1000/30, GUI_redraw, NULL, NULL);
}

void drawCoord()
{
	void *font = GLUT_BITMAP_HELVETICA_18;
	glBegin (GL_LINES);
	glColor3f (1., 0., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (1., 0., 0.);
	glEnd ();
	glRasterPos3d(1., 0., 0.);
	glutBitmapCharacter(font, 'X');

	glBegin (GL_LINES);
	glColor3f (0., 1., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 1., 0.);
	glEnd ();
	glRasterPos3d(0., 1., 0.);
	glutBitmapCharacter(font, 'Y');

	glBegin (GL_LINES);
	glColor3f (0., 0., 1.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 0., 1.);
	glEnd ();
	glRasterPos3d(0., 0., 1.);
	glutBitmapCharacter(font, 'Z');

	gdk_gl_draw_teapot(TRUE, 0.1);
}

void drawOval(float x, float y, float z, float sx, float sy, float sz)
{
	glPushMatrix();
	glColor4f(0, 1, 1, 0.5);
	glTranslatef(x, y, z);
	float _m[] = {
		sx, 0, 0, 0,
		0, sy, 0, 0,
		0, 0, sz, 0,
		0, 0, 0, 1
	};
	glMultMatrixf(_m);
	gdk_gl_draw_sphere(TRUE, 5, 10, 10);
	glPopMatrix();
}

void drawMap(Pts3D &pts, Pts3D &scales)
{
	for(int i=0; i<(int)pts.size(); i++){
		Point3f pt = pts[i];
		Point3f sc;// = scales[i];
		if(scales.empty())
			sc.x=sc.y=sc.z=1;
		else
			sc = scales[i];
		drawOval(pt.x, pt.y, pt.z, sc.x, sc.y, sc.z);
	}
}

int ang = 0;
gboolean GUI::expose (GtkWidget *da, GdkEventExpose *event, gpointer data)
{
	GdkGLContext *glcontext = gtk_widget_get_gl_context (da);
	GdkGLDrawable *gldrawable = gtk_widget_get_gl_drawable (da);
	if (!gdk_gl_drawable_gl_begin (gldrawable, glcontext))
		g_assert_not_reached();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();


	glShadeModel(GL_SMOOTH);

	glRotatef(ang++, 1, 0, 1);
	glScalef (0.5, 0.5, 0.5);
	glRotatef(-0, 1, 0, 0);
	drawCoord();

	glScalef (0.01, 0.01, 0.01);
	gluLookAt(0, 0, 100, 0, 0, 1, 1, 0, 1);
	drawMap(map_pts, map_scales);


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
//	glOrtho (-10,10,-10,10,-20050,10000);
	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	gdk_gl_drawable_gl_end (gldrawable);

	return TRUE;
}

void
GUI::redraw()
{
	struct GUIPacket *p = (struct GUIPacket *) queue->pop();
	if(!p){
		return;
	}
	map_pts = p->pts;
	map_scales = p->scales;
	delete p;

	GtkWidget *da = GTK_WIDGET (glcanvas);

	gdk_window_invalidate_rect (da->window, &da->allocation, FALSE);
	gdk_window_process_updates (da->window, FALSE);
}

