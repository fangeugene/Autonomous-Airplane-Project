// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AAP_IRCamera.cpp - Arduino Library for AAP Infrared WiiMote Camera

	Variables:


	Methods:
		getRawData() : read value from I2C and returns the positions of the
					   IR sources in the image plane coordinate

*/

#include "AAP_IRCamera.h"

// Constructor //////////////////////////////////////////////////////////////
AAP_IRCamera::AAP_IRCamera()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void
AAP_IRCamera::getRawData(Vector2i sources[])
{
	//@Nick: replace the following loop
	for(int i=0; i<4; i++) {
		sources[i].x = 0;
		sources[i].y = 0;
	}
}

Vector3f
AAP_IRCamera::getPosition()
{
	//@Alex: actually compute the position
	Vector3f position;
	position.x = 1;
	position.y = 2;
	position.z = 3;
}
