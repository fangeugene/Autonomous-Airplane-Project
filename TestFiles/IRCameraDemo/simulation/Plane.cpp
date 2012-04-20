#include "Plane.h"

Plane::Plane(const Vector3d& pos, const Vector3d& norm, double s, float c0, float c1, float c2)
	: position(pos)
	, rotation(Matrix3d::Identity())
	, color0(c0)
	, color1(c1)
	, color2(c2)
	, normal(norm)
	, side(s)
{
	rotation_from_tangent(norm.normalized(), rotation);
	setTransform(position, rotation);
	texName[0] = 0;
}

//Image should be saved as BMP 24 bits R8 G8 B8, RGB mode
Plane::Plane(const Vector3d& pos, const Matrix3d& rot, double s)
	: position(pos)
	, rotation(rot)
	, color0(1.0)
	, color1(1.0)
	, color2(1.0)
	, side(s)
{
	setTransform(position, rotation);

  // Makes check images
  int i, j, c; 
  for (i = 0; i < checkImageHeight; i++) {
    for (j = 0; j < checkImageWidth; j++) {
       c = ((((i&0x8)==0)^((j&0x8))==0))*255;
       checkImage[i][j][0] = (GLubyte) c;
       checkImage[i][j][1] = (GLubyte) c;
       checkImage[i][j][2] = (GLubyte) c;
       checkImage[i][j][3] = (GLubyte) 255;
    }
  }
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  glGenTextures(2, texName);
  glBindTexture(GL_TEXTURE_2D, texName[0]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
                 GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                 GL_NEAREST);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth,
              checkImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE,
              checkImage);
}

void Plane::setTransform(const Vector3d& pos, const Matrix3d& rot)
{
	position = pos;
	normal = rot.col(0);
}

void Plane::draw()
{
	Vector3d x = rotation.col(1);
	Vector3d y = rotation.col(2);

	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(color0, color1, color2);
	
	glDisable(GL_CULL_FACE);
	
	bool tex = (texName[0] != 0);
	
	if (tex) {
		//glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		//glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, texName[0]);

		//glTexImage2D (GL_TEXTURE_2D, 0, 3, texture.w, texture.h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, texture.p);
		
		glEnable (GL_TEXTURE_2D);
	}

	glBegin(GL_QUADS);
	if (tex) glTexCoord2s(0, 0);
	glVertex3f(position(0) + side*(x(0)+y(0)), position(1) + side*(x(1)+y(1)), position(2) + side*(x(2)+y(2)));
	if (tex) glTexCoord2s(1, 0);
	glVertex3f(position(0) + side*(x(0)-y(0)), position(1) + side*(x(1)-y(1)), position(2) + side*(x(2)-y(2)));
	if (tex) glTexCoord2s(1, 1);
	glVertex3f(position(0) + side*(-x(0)-y(0)), position(1) + side*(-x(1)-y(1)), position(2) + side*(-x(2)-y(2)));
	if (tex) glTexCoord2s(0, 1);
	glVertex3f(position(0) + side*(-x(0)+y(0)), position(1) + side*(-x(1)+y(1)), position(2) + side*(-x(2)+y(2)));
	glEnd();

	if (tex)
		glDisable (GL_TEXTURE_2D);
	glEnable(GL_CULL_FACE);
	glPopMatrix();
}
