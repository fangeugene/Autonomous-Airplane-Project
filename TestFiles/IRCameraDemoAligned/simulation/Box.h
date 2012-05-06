#ifndef _Box_h
#define _Box_h

#include <stdlib.h>

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/gle.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

class Box
{
	public:
		Box(const Vector3d& pos, const Matrix3d& rot, const Vector3d& half_length_xyz, float c0, float c1, float c2);
		void setTransform(const Vector3d& pos, const Matrix3d& rot);
		void draw();

		Vector3d position;
		Matrix3d rotation;
		Vector3d edge_position;
		float color0;
		float color1;
		float color2;
		Vector3d half_length;
		// needs to be updated ONLY if the position or rotation changes
		vector<Vector3d> normals;
		vector<vector<Vector3d> > vertex_positions;
};

#endif
