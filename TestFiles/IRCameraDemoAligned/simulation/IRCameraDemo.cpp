#include <stdlib.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>

#include <stdarg.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "drawUtils.h"
#include "mathUtils.h"
#include "Mouse.h"
#include "Box.h"
#include "Plane.h"

#include "socketInterfaceReceiver.h"

#include <SerialStream.h>
#include <iostream>
#define PORT "/dev/ttyUSB0" //This is system-specific

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN
using namespace LibSerial;

void drawStuff();
void displayTextInScreen(const char* textline, ...);
void displayTextInScreen(float x, float y, const char* textline, ...);
void bitmap_output(float x, float y, float z, const char* str, void *font);
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
float translate_frame[3] = { 0.0, 0.0, -110.0+20.0 };

double fovy = 61.93;
double z_near = 50.0;
double z_far = 500.0;

GLdouble model_view[16];
GLdouble projection[16];
GLint viewport[4];
int window_width, window_height;

Mouse *mouse0, *mouse1;
Box *servo0, *servo1;
float pan = 0.0, tilt = 0.0;
Box *APM;
Box *wiiMote;
Plane* ground;
double wiiMote_length_x = 3.5;
double wiiMote_length_y = 15.0;
double wiiMote_length_z = 3.0;

bool draw_traj = false;
vector<Vector3d> traj_positions;

SerialStream ardu;

float xx = 0.8;
float yy = 0.95;

void processLeft(int x, int y)
{
	if (mouse0->getKeyPressed() != NONE) {
		mouse0->add2DMove(x-lastx_L, lasty_L-y);
	} else if (mouse1->getKeyPressed() != NONE) {
		mouse1->add2DMove(x-lastx_L, lasty_L-y);
	} else if (/*glutGetModifiers()*/ false &  GLUT_ACTIVE_CTRL){
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
  mouse0->applyTransformFrom2DMove(model_view, projection, viewport);
	mouse1->applyTransformFrom2DMove(model_view, projection, viewport);
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
  if (key == 't')
	  mouse0->setKeyPressed(MOVETAN);
  else if (key == 'm')
    mouse0->setKeyPressed(MOVEPOS);
  else if (key == 'r')
    mouse0->setKeyPressed(ROTATETAN);
  else if (key == 'T')
	  mouse1->setKeyPressed(MOVETAN);
  else if (key == 'M')
    mouse1->setKeyPressed(MOVEPOS);
  else if (key == 'R')
    mouse1->setKeyPressed(ROTATETAN);
  else if (key == '=')
    xx += 0.1;
  else if (key == '-')
    xx -= 0.1;
  else if (key == '+')
    yy += 0.1;
  else if (key == '_')
    yy -= 0.1;
  else if (key == 'd') {
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
  mouse0->setKeyPressed(NONE);
  mouse1->setKeyPressed(NONE);
  mouse0->add2DMove(0.0, 0.0);
  mouse1->add2DMove(0.0, 0.0);
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

double angle(const Vector3d &Va, const Vector3d &Vb, const Vector3d &Vn) {
  double sina = (Va.cross(Vb)).norm();
  double cosa = Va.dot(Vb);
  double angle = atan2( sina, cosa );
  if (Vn.dot(Va.cross(Vb)) < 0)
    angle *= -1;
  return angle;
}

void inverseKinematics(const Matrix3d& reference_frame, const Vector3d& track, float& pan, float& tilt) {
  Vector3d local_track = reference_frame.transpose() * track;
  pan = atan2(-local_track(0), local_track(1)) * 180.0/M_PI;
  tilt = atan2(-local_track(2), sqrt(pow(local_track(0),2.0)+pow(local_track(1),2.0))) * 180.0/M_PI;
  if (pan > 90)
    pan -= 180;
  else if (pan < -90)
    pan += 180;
  else
    tilt *= -1;
  if (tilt < 0)
    tilt += 180;
  if (local_track(2) < 0)
    tilt += 180;
}

void forwardKinematics(const Matrix3d& reference_frame, Vector3d& track, const float& pan, const float& tilt) {
  Matrix3d calc_rotation = ((Matrix3d) Eigen::AngleAxisd(pan*M_PI/180.0, reference_frame.col(2))) * reference_frame;
  calc_rotation = ((Matrix3d) Eigen::AngleAxisd(tilt*M_PI/180.0, calc_rotation.col(0))) * calc_rotation;
  track = calc_rotation.col(1);
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
	
	// Serial read
  vector<string> serial_vect;
  if (ardu.IsOpen()) {
    char str[1024];   
    cout << "serial ";
	  while (true) {
	    ardu >> str;
	    serial_vect.push_back(str);
	    if (strcmp(str,"END")==0) break;
	    cout << str << "   ";
	  }
	  cout << endl;	
	}
	
	ground->draw();
	drawAxes(Vector3d::Zero(), Matrix3d::Identity());
  labelAxes(Vector3d::Zero(), Matrix3d::Identity());
  
  /************** START OF YAW-PITCH-ROLL TESTING **************/
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  if (serial_vect.size() >= 3) {
    yaw   = boost::lexical_cast<double>(serial_vect[0].c_str());
    pitch = boost::lexical_cast<double>(serial_vect[1].c_str());
    roll  = boost::lexical_cast<double>(serial_vect[2].c_str());
  }
  Matrix3d APM_rot = ((Matrix3d) Eigen::AngleAxisd(-yaw*M_PI/180.0, Vector3d::UnitZ()))
                     * ((Matrix3d) Eigen::AngleAxisd(pitch*M_PI/180.0, Vector3d::UnitX()))
                     * ((Matrix3d) Eigen::AngleAxisd(-roll*M_PI/180.0, Vector3d::UnitY()));
  APM->setTransform(APM->position, APM_rot);
  APM->draw();
  drawAxes(APM->position, APM->rotation);
  
  /************** END OF YAW-PITCH-ROLL TESTING **************/
  
  /************** START OF PAN-TILT MOUNT TRACKING TESTING **************/
  // Simulation input and variables from APM
  servo0->setTransform(mouse1->getPosition(), mouse1->getRotation());
  Matrix3d mount_rotation = servo0->rotation;
  Vector3d track = mouse0->getRotation().col(0);
  
  // Tracking algorithm
  float new_pan, new_tilt;
  inverseKinematics(mount_rotation, track, new_pan, new_tilt);
  Vector3d calc_track, new_calc_track;
  forwardKinematics(mount_rotation, calc_track, pan, tilt);
  forwardKinematics(mount_rotation, new_calc_track, new_pan, new_tilt);

  double angle_diff = acos(calc_track.normalized().dot(new_calc_track.normalized())) * 180.0/M_PI;
  if (angle_diff > 10.0) {
    pan = new_pan;
    tilt = new_tilt;
  }
  
  if (pan > 45) pan = 45;
  else if (pan < -45) pan = -45;
  if (tilt > 135) tilt = 135;
  else if (tilt < 45) tilt = 45;
  
  // Simulation and drawing
  glDisable(GL_LIGHTING);
  glColor3f(0.0, 0.0, 0.0);
  displayTextInScreen(0.2, 1.15, "pan %.2f\ttilt %.2f\nnew_pan %.2f\tnew_tilt %.2f\nangle_diff %.2f\n", pan, tilt, new_pan, new_tilt, angle_diff);
  glEnable(GL_LIGHTING);
  
  servo1->rotation = ((Matrix3d) Eigen::AngleAxisd(pan*M_PI/180.0, servo0->rotation.col(2))) * servo0->rotation;
  servo1->rotation = ((Matrix3d) Eigen::AngleAxisd(tilt*M_PI/180.0-M_PI/2.0, servo1->rotation.col(0))) * servo1->rotation;
  servo1->setTransform(servo0->position+2.0*1.75*servo0->rotation.col(2), servo1->rotation);
  mouse0->setTransform(servo1->position, mouse0->getRotation()); // don't change this

  glColor3f(1.0,0.0,0.0);
  drawArrow(servo1->position, 20.0*track);
  glColor3f(0.0,1.0,0.0);
  drawArrow(servo1->position, 20.0*calc_track);
  glColor3f(0.0,0.0,1.0);
  drawArrow(servo1->position, 20.0*new_calc_track);
  servo0->draw();
  drawAxes(servo0->position, servo0->rotation);
  servo1->draw();
  drawAxes(servo1->position, servo1->rotation);
	
	/************** END OF PAN-TILT MOUNT TRACKING TESTING **************/
	
	
	/************** START OF IR LOCALIZATION PART 2 TESTING **************/
	Vector3d t_pos;
	Matrix3d t_rot;
	vector<Vector2d> ir_pos;
	getDeviceState(t_pos, t_rot, ir_pos);
  
  //compute sonar_distance
  /*
  double sonar_distance = - ground->normal.dot(wiiMote->edge_position)/(ground->normal.dot(wiiMote->rotation.col(1)));
  Vector3d intersection = wiiMote->edge_position + sonar_distance * wiiMote->rotation.col(1);
  drawArrow(wiiMote->edge_position, sonar_distance * wiiMote->rotation.col(1));
  
  double height = - sonar_distance * ground->normal.dot(wiiMote->rotation.col(1));
  drawArrow(wiiMote->edge_position, - height * Vector3d::UnitZ());
  */

  // Known fixed separation between ir sources
  double separation = 7.0;
  
  // For every IR source, computed the normalized ray that comes from the camera's origin and crosses the image plane at the pixel location.
  Vector2d ccd_center(512,384);
	double focal_length = 1280;
  vector<Vector3d> test_rays;
	for (int i=0; i<ir_pos.size(); i++) {
	  Vector2d off_center = ir_pos[i] - ccd_center;
  	Vector3d test_ray = t_rot * Vector3d(off_center(0), focal_length, off_center(1));
    test_rays.push_back(test_ray.normalized());
  }
  
  if (ir_pos.size() >= 2) {
    // Each ray intersects the plane at a point. Identify the point that is the closest (miny_index) and the farthest (maxy_index) from the camera.
    int miny_index = 0;
    int maxy_index = 0;
    for (int i=1; i<test_rays.size(); i++) {
      if (test_rays[i](1) < test_rays[miny_index](1))
        miny_index = i;
      if (test_rays[i](1) > test_rays[maxy_index](1))
        maxy_index = i;
    }
    
    // The height from the plane to the camera.
    double height = (ir_pos.size()-1)*separation/(test_rays[maxy_index]/(ground->normal.dot(test_rays[maxy_index])) - test_rays[miny_index]/(ground->normal.dot(test_rays[miny_index]))).norm();
    
    // Adjust the length of the ray such that the length is the distance from the camera's origin to the plane where it intersects.
    for (int i=0; i<test_rays.size(); i++) {
    	double t = - height / (ground->normal.dot(test_rays[i]));
      test_rays[i] = t * test_rays[i];
      /*
      double t = - ground->normal.dot(wiiMote->edge_position)/(ground->normal.dot(wiiMote->rotation.col(1)));
      Vector3d intersection = wiiMote->edge_position + t * wiiMote->rotation.col(1);
      */
    }

	  // Rotation whose Y axis is aligned with the IR sources.
	  Matrix3d aligned_mat;
	  aligned_mat.col(1) = (test_rays[maxy_index] - test_rays[miny_index]).normalized();
	  aligned_mat.col(2) = Vector3d::UnitZ();
	  aligned_mat.col(0) = aligned_mat.col(1).cross(aligned_mat.col(2));
	  
	  Matrix3d calc_rot = aligned_mat.transpose() * t_rot;
	  Vector3d calc_pos = aligned_mat.transpose() * (-test_rays[miny_index]);
	  
	  drawAxes(calc_pos, calc_rot);
    labelAxes(calc_pos, calc_rot);
    glColor3f(0.0,0.0,1.0);
	  drawArrow(calc_pos, aligned_mat.transpose() * test_rays[miny_index]);
	  glColor3d(0.0,0.5,0.5);
	  drawArrow(calc_pos, aligned_mat.transpose() * test_rays[maxy_index]);
	  
	  wiiMote->setTransform(calc_pos + calc_rot * Vector3d(0.0,-wiiMote_length_y/2.0,0.0), calc_rot);
	  wiiMote->draw();
	  drawAxes(wiiMote->edge_position, wiiMote->rotation);
    labelAxes(wiiMote->edge_position, wiiMote->rotation);
    
    cout << "position " << calc_pos.transpose() << endl;
	}
  
  if (draw_traj) {
    traj_positions.push_back(wiiMote->edge_position);
    glColor3f(0.0, 0.5, 0.5);
    for (int i=0; i < traj_positions.size(); i++) {
      drawSphere(traj_positions[i], 1.0);
    }
  }
	
  glPopMatrix();
  
  // Procedure that draws a 2D overlay of the IR sources as seen by the camera.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, 3600, 0, 3600);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);
  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);
  glVertex2i(0, 3600); 
  glVertex2i(0, 3600-768); 
  glVertex2i(1024, 3600-768); 
  glVertex2i(1024, 3600);
  glEnd();
  glColor3f(0, 0, 0);
  glPointSize(2.0);
  glBegin(GL_POINTS);
  for (int i=0; i<ir_pos.size(); i++)
    glVertex2i(ir_pos[i](0), 3600-768+ir_pos[i](1));
  glEnd();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glPopMatrix();
  /************** END OF IR LOCALIZATION PART 2 TESTING **************/

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
	
	mouse0 = new Mouse(Vector3d::Zero(), (Matrix3d) Eigen::AngleAxisd(-M_PI/2.0, Vector3d::UnitZ()));
	mouse1 = new Mouse(Vector3d(0.0, 0.0, 10.0), (Matrix3d) Eigen::AngleAxisd(-M_PI, Vector3d::UnitZ()));

	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

  receiverInit();

  //Serial port initialization
  cout << "Opening serial port " << PORT << endl;
  while (!ardu.IsOpen()) {
    ardu.Open(PORT);
    cout << "Failed to open the port" << endl;
    break;  // Comment out this line to retry the opening
  }
  if (ardu.IsOpen())
    cout << "The port was opened sucessfully" << endl;
  ardu.SetBaudRate(SerialStreamBuf::BAUD_9600);   //The arduino must be setup to use the same baud rate
  ardu.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);

  wiiMote = new Box(Vector3d(0.0,-30.0-wiiMote_length_y/2.0,0.0), Matrix3d::Identity(), Vector3d(wiiMote_length_x/2.0, wiiMote_length_y/2.0, wiiMote_length_z/2.0), 0.0, 0.5, 0.5);
  servo0 = new Box(Vector3d::Zero(), Matrix3d::Identity(), Vector3d(1.5, 1.0, 1.75), 0.2, 0.2, 0.8); // Its transform will be set by the mouse's
  servo1 = new Box(Vector3d::Zero(), Matrix3d::Identity(), Vector3d(1.5, 1.0, 1.75), 0.2, 0.8, 0.2); // Same
  APM = new Box(Vector3d(0.0, 0.0, 20.0), Matrix3d::Identity(), Vector3d(2.5, 5.0, 1.0), 0.0, 0.0, 0.8);
  ground = new Plane(Vector3d(0.0, 0.0, 0.0), (Matrix3d) Eigen::AngleAxisd(M_PI/2.0, -Vector3d::UnitY()), 5*8);

  glutMainLoop ();
}

void displayTextInScreen(float x, float y, const char* textline, ...)
{
  glPushMatrix();
	glLoadIdentity();
  va_list argList;
	char cbuffer[5000];
	va_start(argList, textline);
	vsnprintf(cbuffer, 5000, textline, argList);
	va_end(argList);
	vector<string> textvect;
	boost::split(textvect, cbuffer, boost::is_any_of("\n"));
	float scale = tan(fovy/2.0) * (-z_near*1.002);
	for (int i=0; i<textvect.size(); i++) {
		bitmap_output(x * scale, (y-0.08*i) * scale, -z_near*1.002, textvect[i].c_str(), GLUT_BITMAP_TIMES_ROMAN_24);
	}
  glPopMatrix();
}

void bitmap_output(float x, float y, float z, const char* str, void *font)
{
  int len, i;

  glRasterPos3f(x, y, z);
  len = (int) strlen(str);
  for (i = 0; i < len; i++) {
    glutBitmapCharacter(font, str[i]);
  }
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

