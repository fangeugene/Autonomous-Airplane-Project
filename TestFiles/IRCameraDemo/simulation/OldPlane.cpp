#include "Plane.h"

Plane::Plane(const Vector3d& pos, const Vector3d& norm, double s, float c0, float c1, float c2)
	: position(pos)
	, rotation(Matrix3d::Identity())
	, color0(c0)
	, color1(c1)
	, color2(c2)
	, normal(norm)
	, side(s)
	, file_name("notexture")
{
	rotation_from_tangent(norm.normalized(), rotation);
	setTransform(position, rotation);
}

//Image should be saved as BMP 24 bits R8 G8 B8, RGB mode
Plane::Plane(const Vector3d& pos, const Vector3d& norm, double s, string filename)
	: position(pos)
	, rotation(Matrix3d::Identity())
	, color0(1.0)
	, color1(1.0)
	, color2(1.0)
	, normal(norm)
	, side(s)
	, file_name(filename)
{
	rotation_from_tangent(norm.normalized(), rotation);
	setTransform(position, rotation);

	ilInit();
  if (! LoadImageDevIL ((char*) filename.c_str(), &texture) )
  	cerr << "Failed to load texture from filename " << filename << endl;
}

Plane::~Plane()
{
  //  Clear out the memory used by loading image files.
  if (texture.id)
    ilDeleteImages(1, &texture.id);
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
	
	bool tex = (file_name != "notexture");
	
	if (tex) {
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		glTexImage2D (GL_TEXTURE_2D, 0, 3, texture.w, texture.h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, texture.p);
		
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

ILuint Plane::LoadImageDevIL (char *szFileName, struct TextureHandle *T)
{
    //When IL_ORIGIN_SET enabled, the origin is specified at an absolute 
    //position, and all images loaded or saved adhere to this set origin.
    ilEnable(IL_ORIGIN_SET);
    //sets the origin to be IL_ORIGIN_LOWER_LEFT when loading all images, so 
    //that any image with a different origin will be flipped to have the set 
    //origin.
    ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

    //Now load the image file
    ILuint ImageNameID;
    ilGenImages(1, &ImageNameID);
    ilBindImage(ImageNameID);
    if (!ilLoadImage(szFileName)) return 0; // failure 

    T->id = ImageNameID;
    T->p = ilGetData(); 
    T->w = ilGetInteger(IL_IMAGE_WIDTH);
    T->h = ilGetInteger(IL_IMAGE_HEIGHT);
    
    //printf("%s %d %d %d\n",szFileName,T->id,T->w,T->h);
    return 1; // success
}
