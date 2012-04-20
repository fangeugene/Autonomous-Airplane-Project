/***********************************************************************
SphereFitter2 - Functor plug-in to fit an axis-aligned ellipsoid
(apologies for the misnomer) to a set of points using a Levenberg-
Marquardt minimization algorithm. Uses algebraic distance from point to
ellipsoid; therefore, underestimates the main axis radii.
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

#ifndef SPHEREFITTER2_INCLUDED
#define SPHEREFITTER2_INCLUDED

#include <Math/Math.h>
#include <Geometry/ComponentArray.h>
#include <Geometry/Point.h>
#include <Geometry/Box.h>

class SphereFitter2
	{
	/* Embedded classes: */
	public:
	typedef double Scalar; // Scalar type
	typedef Geometry::Point<Scalar,3> Point; // Type for target points
	typedef Geometry::ComponentArray<Scalar,3> Size; // Type for radius triples
	static const int dimension=6; // Dimension of the optimization space
	typedef Geometry::ComponentArray<Scalar,dimension> Derivative; // Type for distance function derivatives
	
	/* Elements: */
	private:
	unsigned int numPoints; // Number of target points
	const Point* points; // Pointer to array of target points
	Point center; // Current estimated ellipsoid center
	Size radii; // Current estimated ellipsoid radii
	Point centerSave; // Saved estimated ellipsoid center
	Size radiiSave; // Saved estimated ellipsoid radii
	
	/* Constructors and destructors: */
	public:
	SphereFitter2(unsigned int sNumPoints,const Point* sPoints) // Constructs sphere fitter for given set of target points
		:numPoints(sNumPoints),points(sPoints)
		{
		/* Calculate the point's bounding box: */
		Geometry::Box<Scalar,3> box=Geometry::Box<Scalar,3>::empty;
		for(unsigned int i=0;i<numPoints;++i)
			box.addPoint(points[i]);
		for(int i=0;i<3;++i)
			{
			center[i]=Math::mid(box.getMin(i),box.getMax(i));
			radii[i]=Math::div2(box.getMax(i)-box.getMin(i));
			}
		}
	
	/* Methods: */
	void setCenter(const Point& newCenter) // Sets the initial estimate for the sphere's center
		{
		center=newCenter;
		}
	void setRadii(const Size& newRadii) // Sets the initial estimate for the sphere's radius
		{
		radii=newRadii;
		}
	const Point& getCenter(void) const // Returns the estimated center
		{
		return center;
		}
	Size getRadii(void) const // Returns the estimated radius
		{
		return radii;
		}
	void save(void) // Saves the current estimate
		{
		centerSave=center;
		radiiSave=radii;
		}
	void restore(void) // Restores the last saved estimate
		{
		center=centerSave;
		radii=radiiSave;
		}
	unsigned int getNumPoints(void) const // Returns the number of target points
		{
		return numPoints;
		}
	Scalar calcDistance(size_t index) const // Calculates the distance value for the current estimate and the given target point
		{
		return Math::sqrt(Math::sqr((points[index][0]-center[0])/radii[0])+Math::sqr((points[index][1]-center[1])/radii[1])+Math::sqr((points[index][2]-center[2])/radii[2]))-Scalar(1);
		}
	Derivative calcDistanceDerivative(unsigned int index) const // Calculates the derivative of the distance function for the current estimate and the given target point
		{
		Derivative result;
		Scalar comp[3];
		for(int i=0;i<3;++i)
			comp[i]=(points[index][i]-center[i])/radii[i];
		Scalar root=Math::sqrt(Math::sqr(comp[0])+Math::sqr(comp[1])+Math::sqr(comp[2]));
		for(int i=0;i<3;++i)
			{
			result[i]=-comp[i]/root;
			result[3+i]=-Math::sqr(comp[i])/(root*radii[i]);
			}
		return result;
		}
	Scalar calcMag(void) const // Returns the magnitude of the current estimate
		{
		return Math::sqrt(Geometry::sqr(center)+Geometry::sqr(radii));
		}
	void increment(Derivative increment) // Increments the current estimate by the given difference vector
		{
		for(int i=0;i<3;++i)
			{
			center[i]-=increment[i];
			radii[i]-=increment[3+i];
			}
		}
	void normalize(void) // Normalizes the current estimate
		{
		for(int i=0;i<3;++i)
			if(radii[i]<Scalar(0))
				radii[i]=-radii[i];
		}
	};

#endif
