#include <stdlib.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "drawUtils.h"
#include "mathUtils.h"
#include "Box.h"
#include "Plane.h"

#include "socketInterfaceReceiver.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

void drawStuff();
void glutMenu(int ID);
void initGL();

static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};    
static const GLfloat lightOnePosition[] = {140.0, 0.0, 200.0, 0.0};
static const GLfloat lightOneColor[] = {0.5, 0.5, 0.5, 1.0};
static const GLfloat lightTwoPosition[] = {-140.0, 0.0, 200.0, 0.0};
static const GLfloat lightTwoColor[] = {0.9, 0.9, 0.9, 1.0};
static const GLfloat lightThreePosition[] = {140.0, 0.0, -200.0, 0.0};
static const GLfloat lightThreeColor[] = {0.5, 0.5, 0.5, 1.0};
static const GLfloat lightFourPosition[] = {-140.0, 0.0, -200.0, 0.0};
static const GLfloat lightFourColor[] = {0.9, 0.9, 0.9, 1.0};

static GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0 };
static GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0 };
static GLfloat mat_specular[]   = { 0.5f, 0.5f, 0.5f, 1.0 };
static GLfloat high_shininess[] = { 100.0 };

float lastx_L=0;
float lasty_L=0;
float lastx_M=0;
float lasty_M=0;

int pressed_mouse_button;
float rotate_frame[2] = { 0.0, -90.0 };
float translate_frame[3] = { 0.0, 0.0, -110.0 };

double fovy = 61.93;
double z_near = 50.0;
double z_far = 500.0;

GLdouble model_view[16];
GLdouble projection[16];
GLint viewport[4];
int window_width, window_height;

Box* wiiMote;
Plane* ground;
double wiiMote_length_x = 3.5;
double wiiMote_length_y = 15.0;
double wiiMote_length_z = 3.0;

bool draw_traj = false;
vector<Vector3d> traj_positions;

void processLeft(int x, int y)
{
	if (/*glutGetModifiers()*/ false &  GLUT_ACTIVE_CTRL){

		translate_frame[0] += 0.1*(x-lastx_L);
		translate_frame[1] += 0.1*(lasty_L-y);
	} else {
		rotate_frame[0] += 0.2*(x-lastx_L);
		rotate_frame[1] += -0.2*(lasty_L-y);
	}
	lastx_L = x;
	lasty_L = y;
}

void processMiddle(int x, int y)
{
	translate_frame[2] -= x-lastx_M;

	lastx_M = x;
	lasty_M = y;
}

void mouseMotion (int x, int y)
{
  if (pressed_mouse_button == GLUT_LEFT_BUTTON) {
   	processLeft(x, y);
  } else if (pressed_mouse_button == GLUT_MIDDLE_BUTTON) {
    processMiddle(x,y);
  }
	glutPostRedisplay ();
}

void processMouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN) {
    pressed_mouse_button = button;
    if (button == GLUT_LEFT_BUTTON) {
      lastx_L = x;
      lasty_L = y;
    }
    if (button == GLUT_MIDDLE_BUTTON) {
      lastx_M = x;
      lasty_M = y;
    }
	}
}

void processNormalKeys(unsigned char key, int x, int y)
{
  if (key == 'd') {
    draw_traj = !draw_traj;
    if (!draw_traj) {
      traj_positions.clear();
    }
  } else if (key == 'q' || key == 27) {
    exit(0);
  }
  glutPostRedisplay ();
}

void processKeyUp(unsigned char key, int x, int y)
{
}

void processSpecialKeys(int key, int x, int y) {
	switch(key) {
		case GLUT_KEY_LEFT :
				translate_frame[0] += 1;
				break;
		case GLUT_KEY_RIGHT :
				translate_frame[0] -= 1;
				break;
		case GLUT_KEY_UP :
				translate_frame[2] += 1;
				break;
		case GLUT_KEY_DOWN :
				translate_frame[2] -= 1;
				break;
		case GLUT_KEY_PAGE_UP :
				translate_frame[1] -= 1;
				break;
		case GLUT_KEY_PAGE_DOWN :
				translate_frame[1] += 1;
				break;
	}
	glutPostRedisplay ();
}

void processIdle()
{
	//update matrices
	glutPostRedisplay();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(fovy, (GLfloat) w/(GLfloat) h, z_near, z_far);
   glMatrixMode (GL_MODELVIEW);
   glGetIntegerv(GL_VIEWPORT, viewport);
   window_width = viewport[2];
   window_height = viewport[3];
}

void drawStuff()
{
	glPushMatrix ();
	glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(0.4, 0.4, 0.4, 0.0);
  /* set up some matrices so that the object spins with the mouse */
  glTranslatef (translate_frame[0], translate_frame[1], translate_frame[2]);

  glRotatef (rotate_frame[1], 1.0, 0.0, 0.0);
  glRotatef (rotate_frame[0], 0.0, 0.0, 1.0);
  
  glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);
  
	glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
	glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
	glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
	
	Vector3d t_pos;
	Matrix3d t_mat;
	getDeviceState(t_pos, t_mat);
	wiiMote->setTransform(t_pos + t_mat * Vector3d(0.0,-wiiMote_length_y/2.0,0.0), t_mat);
	
	
	// draw
	wiiMote->draw();
	drawAxes(wiiMote->position + wiiMote->rotation * Vector3d(0.0,wiiMote_length_y/2.0,0.0), wiiMote->rotation);
  labelAxes(wiiMote->position + wiiMote->rotation * Vector3d(0.0,wiiMote_length_y/2.0,0.0), wiiMote->rotation);
	
	ground->draw();
	
	glColor3f(1.0, 0.0, 0.0);
	drawSphere(Vector3d(-3.96,0.0,0.0), 1.0);    //Left LED
	drawSphere(Vector3d(0.0,-4.75,3.79), 1.0);   //Top LED
	drawSphere(Vector3d(3.96,0.0,0.0), 1.0);     //Right LED
	drawSphere(Vector3d(0.0,-4.75,-4.47), 1.0);  //Bottom LED
	drawAxes(Vector3d::Zero(), Matrix3d::Identity());
  labelAxes(Vector3d::Zero(), Matrix3d::Identity());
  
  if (draw_traj) {
    traj_positions.push_back(wiiMote->position + wiiMote->rotation * Vector3d(0.0,wiiMote_length_y/2.0,0.0));
    glColor3f(0.0, 0.5, 0.5);
    for (int i=0; i < traj_positions.size(); i++) {
      drawSphere(traj_positions[i], 1.0);
    }
  }
	
  glPopMatrix();

  glutSwapBuffers ();
}

int main (int argc, char * argv[])
{
	/* initialize glut */
	glutInit (&argc, argv); //can i do that?
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	
	glutInitWindowSize(900,900);
	window_width = 900;
	window_height = 900;
	glutCreateWindow ("IR Camera Demo");

	glutPositionWindow(0, 0);

	window_width = glutGet(GLUT_WINDOW_WIDTH);
	window_height = glutGet(GLUT_WINDOW_HEIGHT);
	
	glutReshapeFunc(reshape);
	glutDisplayFunc(drawStuff);
	glutMotionFunc(mouseMotion);
  glutMouseFunc(processMouse);
  glutKeyboardFunc(processNormalKeys);
  glutKeyboardUpFunc(processKeyUp);
  glutSpecialFunc(processSpecialKeys);
  glutIdleFunc(processIdle); 

	/* create popup menu */
	glutCreateMenu (glutMenu);
	glutAddMenuEntry ("Exit", 0);
	glutAttachMenu (GLUT_RIGHT_BUTTON);
	
	initGL();

	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

  receiverInit();

  wiiMote = new Box(Vector3d(0.0,-30.0-wiiMote_length_y/2.0,0.0), Matrix3d::Identity(), Vector3d(wiiMote_length_x/2.0, wiiMote_length_y/2.0, wiiMote_length_z/2.0), 0.0, 0.5, 0.5);
  ground = new Plane(Vector3d(0.0, 0.0, -4.47-2.25), (Matrix3d) Eigen::AngleAxisd(M_PI/2.0, Vector3d::UnitY()), 5*8);

  glutMainLoop ();
}

void initGL()        
{
	// Change background color.
	glClearColor (0.0, 0.0, 0.0, 0.0);
  
  // Enable depth buffering for hidden surface removal.
  //glClearDepth (1.0);
  glDepthFunc(GL_LEQUAL);
	glEnable (GL_DEPTH_TEST);

	// Cull back faces.
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	// Other misc features
	glEnable (GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);

	//glMatrixMode (GL_PROJECTION);
	//glFrustum (-30.0, 30.0, -30.0, 30.0, 50.0, 500.0); // roughly, measured in centimeters
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
  // initialize lighting
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);    
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
  glLightfv (GL_LIGHT0, GL_POSITION, lightOnePosition);
	glLightfv (GL_LIGHT0, GL_DIFFUSE, lightOneColor);
	glEnable (GL_LIGHT0);		// uncomment this if you want another source of light
	glLightfv (GL_LIGHT1, GL_POSITION, lightTwoPosition);
	glLightfv (GL_LIGHT1, GL_DIFFUSE, lightTwoColor);
	glEnable (GL_LIGHT1);
	glLightfv (GL_LIGHT2, GL_POSITION, lightThreePosition);
	glLightfv (GL_LIGHT2, GL_DIFFUSE, lightThreeColor);
	glEnable (GL_LIGHT2); //right
	glLightfv (GL_LIGHT3, GL_POSITION, lightFourPosition);
	glLightfv (GL_LIGHT3, GL_DIFFUSE, lightFourColor);
	glEnable (GL_LIGHT3); //left
	
	glColorMaterial (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable (GL_COLOR_MATERIAL);
	
	glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


void glutMenu(int ID) {
	switch(ID) {
    case 0:
      exit(0);
      break;
  }
}

