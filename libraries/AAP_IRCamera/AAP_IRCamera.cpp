// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AAP_IRCamera.cpp - Arduino Library for AAP Infrared WiiMote Camera

	Variables:


	Methods:
		init(): initializes registers in IR camera
		Write_2bytes(): helper function for writing 2 bytes to I2C slave
		getRawData() : read value from I2C and returns the positions of the
					   IR sources in the image plane coordinate

*/

#include "AAP_IRCamera.h"
#include <I2C.h>

// Constructor //////////////////////////////////////////////////////////////
AAP_IRCamera::AAP_IRCamera()
{
	
}

// Public Methods //////////////////////////////////////////////////////////////
void AAP_IRCamera::init()
{
	_IRsensorAddress = 0xB0;
	_slaveAddress = _IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
	
	I2c.begin();
	// IR sensor initialize
	Write_2bytes(0x30,0x01); delay(10);
	Write_2bytes(0x30,0x08); delay(10);
	Write_2bytes(0x06,0x90); delay(10);
	Write_2bytes(0x08,0xC0); delay(10);
	Write_2bytes(0x1A,0x40); delay(10);
	Write_2bytes(0x33,0x33); delay(10);
	delay(100);
}

void
AAP_IRCamera::getRawData(Vector2i sources[])
{
	I2c.beginTransmission(_slaveAddress);
	I2c.send(0x36);
	I2c.endTransmission();
	
	I2c.requestFrom(_slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
	int i = 0;
	for (i=0;i<16;i++) { _data_buf[i]=0; }
	i = 0;
	while(I2c.available() && i < 16) { 
		_data_buf[i] = I2c.receive();
		i++;
	}
	
	int s = 0;
	
	for(int i=0; i<4; i++) {
		sources[i].x = _data_buf[i*3+1];
		sources[i].y = _data_buf[i*3+2];
		s = _data_buf[i*3+3];
		sources[i].x += (s & 0x30) <<4;
		sources[i].y += (s & 0xC0) <<2;
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

bool AAP_IRCamera::getTransform2(Vector3f& pos, Matrix3f& rot)
{
	Vector2i ir_pos_raw[4];
	getRawData(ir_pos_raw);

	vector<Vector2f> ir_pos;
	for (int i=0; i<4; i++)
		if ((ir_pos_raw[i].x != 1023) && (ir_pos_raw[i].y != 1023))
			ir_pos.push_back(Vector2f((float) ir_pos_raw[i].x, (float) ir_pos_raw[i].y));

	// Known fixed separation between ir sources
	float separation = 153.0;
	Vector3f ground_normal(0.0, 0.0, 1.0);

	// For every IR source, computed the normalized ray that comes from the camera's origin and crosses the image plane at the pixel location.
	Vector2f ccd_center(512,384);
	float focal_length = 1280;
	vector<Vector3f> test_rays;
	for (int i=0; i<ir_pos.size(); i++) {
		Vector2f off_center = ir_pos[i] - ccd_center;
		Vector3f test_ray = rot * Vector3f(off_center.x, focal_length, off_center.y);
		test_rays.push_back(test_ray.normalized());
	}

	if (ir_pos.size() >= 2) {
		// Each ray intersects the plane at a point. Identify the point that is the closest (miny_index) and the farthest (maxy_index) from the camera.
		int minx_index = 0;
		int maxx_index = 0;
		for (int i=1; i<test_rays.size(); i++) {
			if (test_rays[i].x < test_rays[minx_index].x)
				minx_index = i;
			if (test_rays[i].x > test_rays[maxx_index].x)
				maxx_index = i;
		}

		// The height from the plane to the camera.
		float height = (ir_pos.size()-1)*separation/(test_rays[maxx_index]/(ground_normal * test_rays[maxx_index]) - test_rays[minx_index]/(ground_normal * test_rays[minx_index])).length();

	    // Adjust the length of the ray such that the length is the distance from the camera's origin to the plane where it intersects.
	    for (int i=0; i<test_rays.size(); i++) {
	    	float t = - height / (ground_normal * test_rays[i]);
	    	test_rays[i] = test_rays[i] * t;
	    }

	    // Rotation whose Y axis is aligned with the IR sources.
	    Matrix3f aligned_mat;
	    aligned_mat.setCol(0, (test_rays[maxx_index] - test_rays[minx_index]).normalized());
	    aligned_mat.setCol(2, Vector3f(0.0, 0.0, 1.0));
	    aligned_mat.setCol(1, aligned_mat.col(2) % (aligned_mat.col(0)));

	    rot = aligned_mat.transposed() * rot;
	    pos = aligned_mat.transposed() * (-(test_rays[maxx_index] + test_rays[minx_index])/2.0);

	    return true;
	}
	return false;
}

// Private Methods //////////////////////////////////////////////////////////////

void AAP_IRCamera::Write_2bytes(byte d1, byte d2)
{
	I2c.beginTransmission(_slaveAddress);
	I2c.send(d1); I2c.send(d2);
	I2c.endTransmission();
}


