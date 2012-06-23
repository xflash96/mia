#include <GLUT/glut.h>
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <unistd.h>		 // Header File For sleeping.

#define ESCAPE 27

int window; 
float rquad = 0.0f;

void InitGL(int Width, int Height)					// We call this right after our OpenGL window is created.
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS);				// The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST);			// Enables Depth Testing
	glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();				// Reset The Projection Matrix

	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

	glMatrixMode(GL_MODELVIEW);
}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(int Width, int Height)
{
	if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;

	glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	glTranslatef(0.0f,0.0f,7.0f);		// Move Left 1.5 Units And Into The Screen 6.0

	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
}

/* The main drawing function. */
void DrawGLScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear The Screen And The Depth Buffer
	glLoadIdentity();				// Reset The View
	gluLookAt(5, 0, 5, 3, 0, -3, 0, 1, 0);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(-50, -2, -20);
	glVertex3f(50, -2, -20);

	glColor3f(0, 1, 0);
	glVertex3f(0, -50, -20);
	glVertex3f(0, 50, -20);

	glColor3f(0, 0, 1);
	glVertex3f(0, -2, -100);
	glVertex3f(0, -2, 10);
	glEnd();
  	glLoadIdentity();				// Reset The View
  	glTranslatef(-1.5f,0.0f,-6.0f);		// Move Left 1.5 Units And Into The Screen 6.0
	//glBegin(GL_POLYGON);				// start drawing a polygon
	glColor3f(1, 1, 1);
	//glutWireCube(1);
  	glScalef(0.5, 1, 0.5);
  	glutSolidSphere(1, 50, 40);
	//glEnd();					// we're done with the polygon
	
	glutSwapBuffers();
}

/* The function called whenever a key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{
	usleep(100);

	if (key == ESCAPE) 
	{ 
		glutDestroyWindow(window); 
		exit(0);									 
	}
}

int main(int argc, char **argv) 
{	
	glutInit(&argc, argv);	

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);	

	glutInitWindowSize(640, 480);	

	glutInitWindowPosition(0, 0);	

	window = glutCreateWindow("Jeff Molofee's GL Code Tutorial ... NeHe '99");	

	glutDisplayFunc(&DrawGLScene);	

	glutFullScreen();

	glutIdleFunc(&DrawGLScene);

	glutReshapeFunc(&ReSizeGLScene);

	glutKeyboardFunc(&keyPressed);

	InitGL(640, 480);
	
	glutMainLoop();	

	return 1;
}
