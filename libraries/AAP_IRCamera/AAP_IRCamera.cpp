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
	_IRsensorAddress = 0xB0;
	_slaveAddress = _IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
}

// Public Methods //////////////////////////////////////////////////////////////
void IRsensor::init()
{
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


// Private Methods //////////////////////////////////////////////////////////////

void IRsensor::Write_2bytes(byte d1, byte d2)
{
	I2c.beginTransmission(_slaveAddress);
	I2c.send(d1); I2c.send(d2);
	I2c.endTransmission();
}


