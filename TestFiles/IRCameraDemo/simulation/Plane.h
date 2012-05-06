#ifndef _Plane_h
#define _Plane_h

#include <stdlib.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

#include "mathUtils.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

#define	checkImageWidth 128
#define	checkImageHeight 128

class Plane
{
	public:
		Plane(const Vector3d& pos, const Vector3d& norm, double s, float c0, float c1, float c2);
		Plane(const Vector3d& pos, const Matrix3d& rot, double s);
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();
	
	protected:
  	Vector3d position;
  	Matrix3d rotation;
		float color0;
		float color1;
		float color2;
		Vector3d normal;
		double side;

		GLubyte checkImage[checkImageHeight][checkImageWidth][4];

    GLuint texName[1];
};

#endif
