/***********************************************************************
SimpleWiiCameraTest - Non-visual test program for 3D tracking of the Wii
controller using its accelerometers and IR camera.
Copyright (c) 2008 Oliver Kreylos

This file is part of the Wii Controller Tracking Package.

The Wii Controller Tracking Package is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Wii Controller Tracking Package is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Wii Controller Tracking Package; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

/* Need this to generate code for 7-vectors and 7x7 matrices: */
#define NONSTANDARD_TEMPLATES

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <Threads/Mutex.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/OrthonormalTransformation.h>

// Time
#include <sys/time.h>

#include "Wiimote.h"
#include "CameraFitter.h"
#include "LevenbergMarquardtMinimizer2.h"

#include "Sockets/socketInterfaceSender.h"

/****************
Helper functions:
****************/

template <class ScalarParam,int dimensionParam>
std::ostream& operator<<(std::ostream& os,const Geometry::ComponentArray<ScalarParam,dimensionParam>& ca)
	{
	std::ios::fmtflags oldFlags=os.flags();
	std::streamsize oldPrecision=os.precision();
	os.setf(std::ios::fixed);
	os<<"("<<std::setprecision(4)<<std::setw(8)<<ca[0];
	for(int i=1;i<dimensionParam;++i)
		os<<", "<<std::setprecision(4)<<std::setw(8)<<ca[i];
	os<<")";
	os.flags(oldFlags);
	os.precision(oldPrecision);
	return os;
	}

std::ostream& operator<<(std::ostream& os,const CameraFitter::Transform& t)
	{
	os<<t.getOrigin()<<", "<<t.getDirection(0)<<", "<<t.getDirection(1)<<", "<<t.getDirection(2);
	return os;
	}

/* Type declarations: */
typedef CameraFitter::Scalar Scalar; // Scalar type for points and vectors
typedef CameraFitter::Point Point; // Type for 3D points
typedef CameraFitter::Vector Vector; // Type for 3D vectors
typedef CameraFitter::Transform Transform; // Type for 3D rigid body transformations
typedef Transform::Rotation Rotation; // Type for 3D rotations / orientations
typedef CameraFitter::Pixel Pixel; // Type for pixels on the Wiimote's CCD
typedef LevenbergMarquardtMinimizer2<CameraFitter> LMCamera; // Class to solve for Wiimote's position and orientation

int main(int argc,char* argv[])
	{
	/* Parse the command line: */
	const char* wiimoteName=0; // Use default Wiimote name
	Scalar posFilterCoefficient=Scalar(0.1); // 1.0 disables filtering; values approaching 0.0 create very low-pass filters
	for(int i=1;i<argc;++i)
		{
		if(strcasecmp(argv[i],"-filter")==0)
			{
			if(i<argc-1)
				{
				/* Read the filter coefficient: */
				++i;
				double fc=atof(argv[i]);
				if(fc>0.0&&fc<=1.0)
					posFilterCoefficient=Scalar(fc);
				else
					std::cerr<<"Filter coefficient outside valid range (0.0 - 1.0]"<<std::endl;
				}
			else
				std::cerr<<"Ignoring dangling -filter option"<<std::endl;
			}
		else if(wiimoteName==0)
			{
			/* Remember the name of the Wiimote device: */
			wiimoteName=argv[i];
			}
		else
			std::cerr<<"Ignoring command line parameter "<<argv[i]<<std::endl;
		}
	
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
	
	/* Connect to the Wiimote device: */
	Wiimote* wiimote=0;
	if(argc>=2)
		{
		std::cout<<"Connecting to bluetooth device "<<argv[1]<<"."<<std::endl;
		std::cout<<"Please press buttons 1 and 2 to initiate connection..."<<std::flush;
		wiimote=new Wiimote(argv[1]);
		std::cout<<" done"<<std::endl;
		}
	else
		{
		std::cout<<"Connecting to first Wiimote device in range"<<std::endl;
		std::cout<<"Please press buttons 1 and 2 to initiate connection..."<<std::flush;
		wiimote=new Wiimote(0);
		std::cout<<" done"<<std::endl;
		}
	
	/* Set up the Wiimote data reports for IR tracking: */
	wiimote->requestAccelerometers(true);
	wiimote->requestIRTracking(true);
	
	/* Initialize the Wiimote's state: */
	wiimote->waitForEvent();
	Vector acceleration=wiimote->getAcceleration(0); // Initial acceleration vector
	Vector lastAcceleration=acceleration;
	Transform wiipos=wiiCamera.getTransform(); // Initial position and orientation as set in the camera fitter
	
	// Time
	timeval start, end;
  long useconds;
	
	senderInit();
	
	/* Process data packets from the Wiimote until interrupted: */
	while(true)
		{
		/* Wait for the next data packet: */
		wiimote->waitForEvent();
		
		// Time
		gettimeofday(&start, NULL);
		
		/* Read the current instantaneous acceleration vector: */
		Vector newAcceleration=wiimote->getAcceleration(0);
		
		/* Update the filtered acceleration vector: */
		Vector da=newAcceleration-lastAcceleration;
		Scalar trust=Math::exp(-Geometry::sqr(da)*Scalar(50))*Scalar(0.2); // Simple trust function that prefers steady acceleration vectors
		acceleration+=(newAcceleration-acceleration)*trust;
		lastAcceleration=newAcceleration;
		
		/* Calculate an intermediate orientation based on the filtered acceleration vector: */
		Vector previousY=wiipos.getDirection(1);
		Scalar yaw=Math::acos(previousY[1]/Math::sqrt(Math::sqr(previousY[0])+Math::sqr(previousY[1])));
		if(previousY[0]>Scalar(0))
			yaw=-yaw;
		Scalar axz=Math::sqrt(Math::sqr(acceleration[0])+Math::sqr(acceleration[2]));
		Scalar roll=Math::acos(acceleration[2]/axz);
		if(acceleration[0]>Scalar(0))
			roll=-roll;
		Scalar pitch=Math::acos(axz/Math::sqrt(Math::sqr(acceleration[1])+Math::sqr(axz)));
		if(acceleration[1]<Scalar(0))
			pitch=-pitch;
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
				PVector vy(acceleration[0],acceleration[2]);
				vy.normalize();
				PVector vx=-Geometry::normal(vy);
				vx.normalize();
				
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
			
			gettimeofday(&end, NULL);
      useconds = end.tv_usec - start.tv_usec;
		
		/* Print the new Wiimote position and orientation: */
		//std::cout<<"\r"<<useconds<<"\t"<<wiipos<<std::flush;
		
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
		}
	std::cout<<std::endl;
	
	/* Clean up and exit: */
	delete wiimote;
	return 0;
	}
