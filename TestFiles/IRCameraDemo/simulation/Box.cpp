#include "Box.h"

Box::Box(const Vector3d& pos, const Matrix3d& rot, const Vector3d& half_length_xyz, float c0, float c1, float c2)
	: position(pos)
	, rotation(rot)
	, color0(c0)
	, color1(c1)
	, color2(c2)
	, half_length(half_length_xyz)
{
  setTransform(position, rotation);
}

void Box::setTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	rotation = rot;

	normals.resize(6);
	vertex_positions.resize(6);
	for (int axis_sign_ind = 0; axis_sign_ind < 2; axis_sign_ind++) {
		for (int axis = 0; axis < 3; axis++) {
			float axis_sign = (axis_sign_ind) ? -1 : 1;
			Vector3d vertex_sign[4];
			vertex_sign[0][axis] = vertex_sign[1][axis] = vertex_sign[2][axis] = vertex_sign[3][axis] = axis_sign;
			vertex_sign[0][(axis+1)%3] = vertex_sign[0][(axis+2)%3] = 1;
			vertex_sign[2][(axis+1)%3] = vertex_sign[2][(axis+2)%3] = -1;			
			vertex_sign[1][(axis+2)%3] = vertex_sign[3][(axis+1)%3] = axis_sign * 1;
			vertex_sign[1][(axis+1)%3] = vertex_sign[3][(axis+2)%3] = axis_sign * (-1);

			normals[axis_sign_ind*3 + axis] = axis_sign * rotation.col(axis);
			vertex_positions[axis_sign_ind*3 + axis].resize(4);
			for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++) {
				vertex_positions[axis_sign_ind*3 + axis][vertex_ind] = position + 
																															 vertex_sign[vertex_ind][0]*half_length(0)*rotation.col(0) + 
																															 vertex_sign[vertex_ind][1]*half_length(1)*rotation.col(1) + 
																															 vertex_sign[vertex_ind][2]*half_length(0)*rotation.col(2);
			}
		}
	}
}

void Box::draw()
{
	glPushMatrix();
	glColor3f(color0, color1, color2);
	for (int face_ind = 0; face_ind < 6; face_ind++) {
		for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++) {
			glBegin(GL_QUADS);
			glNormal3f(normals[face_ind](0), normals[face_ind](1), normals[face_ind](2));
			for (int vertex_ind = 0; vertex_ind < 4; vertex_ind++)
			{
				glVertex3f(vertex_positions[face_ind][vertex_ind](0), vertex_positions[face_ind][vertex_ind](1), vertex_positions[face_ind][vertex_ind](2));
			}
			glEnd();
		}
	}
	glPopMatrix();
}
