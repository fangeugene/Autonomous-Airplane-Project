/***********************************************************************
LevenbergMarquardtTest - Test program for the Levenberg-Marquardt
minimization algorithm.
Copyright (c) 2007-2008 Oliver Kreylos

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

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <Misc/File.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Random.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/Random.h>

#define NONSTANDARD_TEMPLATES
#include "SphereFitter.h"
#include "CylinderFitter.h"
#include "LevenbergMarquardtMinimizer2.h"

typedef LevenbergMarquardtMinimizer2<SphereFitter> LMSphere; // L-M fitter for spheres
typedef LevenbergMarquardtMinimizer2<CylinderFitter> LMCylinder; // L-M fitter for cylinders

void testSphere(const SphereFitter::Point& center,SphereFitter::Scalar radius,SphereFitter::Scalar error,unsigned int numPoints)
	{
	/* Sample the sphere (with some added noise): */
	SphereFitter::Point* points=new SphereFitter::Point[numPoints];
	for(unsigned int i=0;i<numPoints;++i)
		{
		/* Create a random point on the sphere's surface: */
		points[i]=center+Geometry::randVectorUniform<SphereFitter::Scalar,SphereFitter::Point::dimension>(radius);
		
		/* Offset the point by a normally-distributed error vector: */
		points[i]+=Geometry::randVectorNormal<SphereFitter::Scalar,SphereFitter::Point::dimension>(error);
		}
	
	/* Initialize the sphere fitter: */
	SphereFitter sf(numPoints,points);
	sf.setCenter(SphereFitter::Point(0,0,0));
	sf.setRadius(SphereFitter::Scalar(1));
	sf.normalize();
	
	/* Create a Levenberg-Marquardt minimizer with default parameters: */
	LMSphere lms;
	
	/* Run the Levenberg-Marquardt minimizer: */
	lms.minimize(sf);
	
	/* Print the minimization result: */
	std::cout<<"("<<sf.getCenter()[0]<<", "<<sf.getCenter()[1]<<", "<<sf.getCenter()[2]<<"), "<<sf.getRadius()<<std::endl;
	}

void testCylinder(const CylinderFitter::Point& center,const CylinderFitter::Vector& axis,CylinderFitter::Scalar radius,CylinderFitter::Scalar error,unsigned int numPoints)
	{
	/* Sample the cylinder (with some added noise): */
	CylinderFitter::Scalar axisLen=Geometry::mag(axis);
	CylinderFitter::Vector x=Geometry::normal(axis);
	x.normalize();
	CylinderFitter::Vector y=Geometry::cross(axis,x);
	y.normalize();
	CylinderFitter::Point* points=new CylinderFitter::Point[numPoints];
	for(unsigned int i=0;i<numPoints;++i)
		{
		/* Create a random point on the cylinder's surface: */
		double angle=Math::randUniformCO(0.0,2.0*Math::Constants<double>::pi);
		double elevation=Math::randUniformCC(-radius/axisLen,radius/axisLen);
		points[i]=center+axis*elevation+x*Math::cos(angle)*radius+y*Math::sin(angle)*radius;
		
		/* Offset the point by a normally-distributed error vector: */
		points[i]+=Geometry::randVectorNormal<SphereFitter::Scalar,SphereFitter::Point::dimension>(error);
		}
	
	/* Initialize the cylinder fitter: */
	CylinderFitter cf(numPoints,points);
	
	/* Create a Levenberg-Marquardt minimizer with default parameters: */
	LMCylinder lmc;
	
	/* Run the Levenberg-Marquardt minimizer: */
	lmc.minimize(cf);
	
	/* Print the minimization result: */
	std::cout<<"("<<cf.getCenter()[0]<<", "<<cf.getCenter()[1]<<", "<<cf.getCenter()[2]<<"), ("<<cf.getAxis()[0]<<", "<<cf.getAxis()[1]<<", "<<cf.getAxis()[2]<<"), "<<cf.getRadius()<<std::endl;
	}

void testSavedData(const char* filename)
	{
	/* Open the point file: */
	Misc::File pointFile(filename,"rt");
	std::vector<CylinderFitter::Point> points;
	while(!pointFile.eof())
		{
		char line[256];
		pointFile.gets(line,sizeof(line));
		
		double p[3];
		sscanf(line,"%lf %lf %lf",&p[0],&p[1],&p[2]);
		points.push_back(CylinderFitter::Point(p));
		}
	
	/* Initialize the cylinder fitter: */
	CylinderFitter cf(points.size(),&points[0]);
	
	/* Create a Levenberg-Marquardt minimizer with default parameters: */
	LMCylinder lmc;
	
	/* Run the Levenberg-Marquardt minimizer: */
	lmc.minimize(cf);
	
	/* Print the minimization result: */
	std::cout<<"("<<cf.getCenter()[0]<<", "<<cf.getCenter()[1]<<", "<<cf.getCenter()[2]<<"), ("<<cf.getAxis()[0]<<", "<<cf.getAxis()[1]<<", "<<cf.getAxis()[2]<<"), "<<cf.getRadius()<<std::endl;
	}

int main(void)
	{
	srand(time(0));
	
	/* Fit a sphere: */
	// testSphere(SphereFitter::Point(20,-10,30),SphereFitter::Scalar(20),SphereFitter::Scalar(0.5),10000);
	
	/* Fit a cylinder: */
	testCylinder(CylinderFitter::Point(20,-10,30),CylinderFitter::Vector(0.0,0.0,1.0),CylinderFitter::Scalar(10),SphereFitter::Scalar(1.0),100000);
	
	// testSavedData("SelectedPoints.xyzi");
	
	return 0;
	}
