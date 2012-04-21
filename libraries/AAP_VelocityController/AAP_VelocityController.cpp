// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AAP_VelocityController.cpp - Arduino Library for AAP Velocity Controller

	Variables:


	Methods:
		getOutput(float setpoint, float altitude, float kP, float kD) : return motor output as a float

*/

#include "AAP_VelocityController.h"
#include "WProgram.h"

// vars
volatile float lastOutput;
volatile float lastAltitude;
volatile float ret;

// Constructor //////////////////////////////////////////////////////////////
AAP_VelocityController::AAP_VelocityController()
{
	lastOutput = 0;
	lastAltitude = 0;
	ret = 0;
}
// Public Methods //////////////////////////////////////////////////////////////
float
AAP_VelocityController::getOutput(float setpoint, float altitude, float kP, float kD)
{
	ret = lastOutput + (setpoint - altitude)*kP - (altitude - lastAltitude)*kD;
	if (ret > 100) {
		ret = 100;
	} else if (ret < 0) {
		ret = 0;
	}
	
	lastOutput = ret;
	lastAltitude = altitude;
	//ret = ret + 0.1;
	return ret;
}