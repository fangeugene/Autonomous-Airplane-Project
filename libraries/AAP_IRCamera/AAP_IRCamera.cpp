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
#include "CameraFitter.h"
#include <I2C.h>
#include <../Geometry/Point.h>
#include <../Geometry/Vector.h>
#include <../Geometry/AffineCombiner.h>
#include <../Geometry/OrthonormalTransformation.h>
#include <../AP_DCM/AP_DCM.h>         // ArduPilot Mega DCM Library

/* Type declarations: */
typedef CameraFitter::Scalar Scalar; // Scalar type for points and vectors
typedef CameraFitter::Point Point; // Type for 3D points
typedef CameraFitter::Vector Vector; // Type for 3D vectors
typedef CameraFitter::Transform Transform; // Type for 3D rigid body transformations
typedef Transform::Rotation Rotation; // Type for 3D rotations / orientations
typedef CameraFitter::Pixel Pixel; // Type for pixels on the Wiimote's CCD
typedef LevenbergMarquardtMinimizer2<CameraFitter> LMCamera; // Class to solve for Wiimote's position and orientation

static GPS *g_gps;
AP_GPS_None g_gps_driver(NULL);

static AP_ADC_ADS7844          adc;
AP_InertialSensor_Oilpan ins( &adc );
AP_IMU_INS imu(&ins, 0);
AP_DCM dcm(&imu, g_gps);

// Constructor //////////////////////////////////////////////////////////////
AAP_IRCamera::AAP_IRCamera()
{
	_IRsensorAddress = 0xB0;
	_slaveAddress = _IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
	
	/* Create a camera fitter object for IR tracking: */
	Pixel ccdCenter=Pixel(512,384);
	Scalar focalLength=Scalar(1280);
	CameraFitter wiiCamera(ccdCenter,focalLength); // CCD center and focal length estimated rule-of-thumb; seem to work
	
	/* Initialize the positions of the four LEDs on the custom beacon (need to be measured from actual beacon): */
	Point targetPoints[4]=
		{
		Point(-3.96,0.0,0.0),  // Left LED
		Point(0.0,-4.75,3.79),       // Top LED
		Point(3.96,0.0,0.0),   // Right LED
		Point(0.0,-4.75,-4.47) // Bottom LED
		};
	
	/* Transformation to convert LED positions from beacon coordinates to world coordinates: */
	Transform targetTransform=Transform::identity;
	//targetTransform*=Transform::translate(Vector(0.0,0.0,7.5));
	//targetTransform*=Transform::rotate(Transform::Rotation::rotateX(Math::rad(37.5)));
	//targetTransform*=Transform::translate(Vector(0.0,0.0,2.75));
	
	/* Set LED positions in world coordinates in camera fitter object: */
	for(int i=0;i<4;++i)
		wiiCamera.setTargetPoint(i,targetTransform.transform(targetPoints[i]));
	
	/* Set the Wiimote's "home" transformation; should be approximately the position it's held at program startup: */
	wiiCamera.setTransform(Transform::translate(Vector(0,-30,0)));
	
	/* Create valid flags for all LEDs: */
	bool pixelValids[4];
	for(int i=0;i<4;++i)
		pixelValids[i]=false;
	
	/* Create a map from CCD LED indices to target LED indices: */
	int pixelMap[4];
	for(int i=0;i<4;++i)
		pixelMap[i]=i;
	
	/* Create an array of pixel positions: */
	Pixel pixels[4];
	
	/* Create a Levenberg-Marquardt minimizer: */
	LMCamera lmc;
}

// Public Methods //////////////////////////////////////////////////////////////
void AAP_IRCamera::init()
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
	
	//TODO make sure these are needed
	imu.init(IMU::COLD_START, delay, flash_leds, &timer_scheduler);
  	dcm.matrix_reset();
	
	
}

void AAP_IRCamera::getRawData()
{
	Vector2i sources[];
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
	return sources;
}

Vector3f AAP_IRCamera::getPosition() {
	dcm.update_DCM();
	int yaw = (uint16_t)dcm.yaw_sensor / 100;
	int pitch = (uint16_t)dcm.pitch_sensor / 100;
	int roll = (uint16_t)dcm.roll_sensor / 100;
	Rotation wiirot=Rotation::rotateZ(yaw);
	wiirot*=Rotation::rotateX(pitch);
	wiirot*=Rotation::rotateY(roll);
		
	/* Update the wiimote's orientation based on the acceleration vector only: */
	wiipos=Transform(wiipos.getTranslation(),wiirot);
	
	/* Process the IR camera targets: */
	int numValidTargets=0;
	Pixel::AffineCombiner pixelCentroid; // To calculate the centroid of all visible LED targets
	for(int i=0;i<4;++i)
		{
		pixelValids[i]=wiimote->getIRTargetValid(i);
		if(pixelValids[i])
			{
			pixels[i][0]=Scalar(wiimote->getIRTargetX(i));
			pixels[i][1]=Scalar(wiimote->getIRTargetY(i));
			++numValidTargets;
			pixelCentroid.addPoint(pixels[i]);
			}
		}
	
	if(numValidTargets>0) // Don't use IR tracking if no LEDs are visible
		{
		/*****************************************************************
		The following pixel matching code is highly experimental; better
		approaches are needed to make tracking more robust.
		*****************************************************************/
		
		if(numValidTargets==4) // If all LEDs are visible, match based on orientation derived from acceleration
			{
			/* Project the "up" vector into camera space: */
			typedef Geometry::Vector<Scalar,2> PVector;
			PVector vx(wiirot.getDirection(0)[0], wiirot.getDirection(2)[0]);
			PVector vy = Geometry::normal(vx);
			vy.normalize();
			
			/* Find the leftmost, rightmost, and topmost points (this sort of depends on the beacon's shape): */
			Scalar minX,maxX,minY,maxY;
			int minXIndex,maxXIndex,minYIndex,maxYIndex;
			minX=minY=Math::Constants<Scalar>::max;
			maxX=maxY=Math::Constants<Scalar>::min;
			minXIndex=maxXIndex=minYIndex=maxYIndex=-1;
			for(int i=0;i<4;++i)
				{
				Scalar x=pixels[i]*vx;
				Scalar y=pixels[i]*vy;
				if(minX>x)
					{
					minX=x;
					minXIndex=i;
					}
				if(maxX<x)
					{
					maxX=x;
					maxXIndex=i;
					}
				if(minY>y)
					{
					minY=y;
					minYIndex=i;
					}
				if(maxY<y)
					{
					maxY=y;
					maxYIndex=i;
					}
				}
			
			/* Create the pixel-target map: */
			pixelMap[minXIndex]=0;
			pixelMap[maxYIndex]=1;
			pixelMap[maxXIndex]=2;
			for(int i=0;i<4;++i)
				if(i!=minXIndex&&i!=maxYIndex&&i!=maxXIndex)
					pixelMap[i]=3;
			}
		else // Not all LEDs are visible
			{
			/* Project the target points into camera space using the previous camera position/orientation and match closest pairs: */
			wiiCamera.setTransform(wiipos);
			for(int pixelIndex=0;pixelIndex<4;++pixelIndex)
				if(pixelValids[pixelIndex])
					{
					Scalar minDist2=Geometry::sqrDist(pixels[pixelIndex],wiiCamera.project(0));
					int minIndex=0;
					for(int i=1;i<4;++i)
						{
						Scalar dist2=Geometry::sqrDist(pixels[pixelIndex],wiiCamera.project(i));
						if(minDist2>dist2)
							{
							minDist2=dist2;
							minIndex=i;
							}
						}
					pixelMap[pixelIndex]=minIndex;
					}
			}
		
		/* Re-project the new pixel positions using the camera fitter and a Levenberg-Marquardt minimizer: */
		for(int i=0;i<4;++i)
			wiiCamera.invalidatePixel(i);
		for(int i=0;i<4;++i)
			if(pixelValids[i])
				wiiCamera.setPixel(pixelMap[i],pixels[i]);
		lmc.minimize(wiiCamera);
		
		if(posFilterCoefficient<Scalar(1))
			{
			/* Low-pass filter the reconstructed camera transformation: */
			Transform deltaWP=Geometry::invert(wiipos);
			deltaWP.leftMultiply(wiiCamera.getTransform());
			Vector t=deltaWP.getTranslation();
			t*=posFilterCoefficient;
			Vector r=deltaWP.getRotation().getScaledAxis();
			r*=posFilterCoefficient;
			deltaWP=Transform(t,Rotation::rotateScaledAxis(r));
			wiipos.leftMultiply(deltaWP);
			wiipos.renormalize();
			}
		else
			wiipos=wiiCamera.getTransform();
		}
		
		
		/* Print the new Wiimote position and orientation: */
		//std::cout<<"\r"<<useconds<<"\t"<<wiipos<<std::flush;
		
		//TODO return this properly
		/*
		sendDeviceState(wiipos.getOrigin()[0],
                    wiipos.getOrigin()[1],
		                wiipos.getOrigin()[2],
		                wiipos.getDirection(0)[0],
		                wiipos.getDirection(0)[1],
		                wiipos.getDirection(0)[2],
		                wiipos.getDirection(1)[0],
		                wiipos.getDirection(1)[1],
		                wiipos.getDirection(1)[2],
		                wiipos.getDirection(2)[0],
		                wiipos.getDirection(2)[1],
		                wiipos.getDirection(2)[2]);
		                */
		return Vector3f(wiipos.getOrigin()[0],wiipos.getOrigin()[1],wiipos.getOrigin()[2]);
}


// Private Methods //////////////////////////////////////////////////////////////

void AAP_IRCamera::Write_2bytes(byte d1, byte d2)
{
	I2c.beginTransmission(_slaveAddress);
	I2c.send(d1); I2c.send(d2);
	I2c.endTransmission();
}


