/***********************************************************************
WiiTest - Test program for the Wiimote class. Connects to Wiimote of
given name or address (or first Wiimote in range if no command line
parameters are given) and prints the button and accelerometer status of
the device and an optional nunchuck extension.
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

#include <iostream>

#include "Wiimote.h"

int main(int argc,char* argv[])
	{
	Wiimote wiimote(argc>=2?argv[1]:0);
	// wiimote.requestContinuousReports(true);
	wiimote.requestAccelerometers(false);
	wiimote.requestIRTracking(false);
	
	const int historySize=1024;
	int accelHistory[3][historySize]; // History array for raw accelerometer values
	for(int i=0;i<3;++i)
		for(int j=0;j<historySize;++j)
			accelHistory[i][j]=0;
	int historyIndex=0;
	while(true)
		{
		wiimote.waitForEvent();
		
		/* Read and store raw accelerometer values: */
		for(int i=0;i<3;++i)
			accelHistory[i][historyIndex]=wiimote.getRawAccelerometerValue(i);
		++historyIndex;
		if(historyIndex==historySize)
			{
			#if 0
			/* Print the average raw accelerometer values: */
			std::cout<<std::endl;
			for(int i=0;i<3;++i)
				{
				double average=0.0;
				for(int j=0;j<historySize;++j)
					average+=double(accelHistory[i][j]);
				average/=double(historySize);
				std::cout<<average<<"   ";
				}
			std::cout<<std::endl;
			#endif
			historyIndex=0;
			}
		
		std::cout<<"\r";
		int numButtons=wiimote.getNumButtons();
		for(int i=0;i<numButtons;++i)
			{
			if(wiimote.getButtonState(i))
				std::cout<<"X ";
			else
				std::cout<<". ";
			}
		
		if(wiimote.getExtensionDevice()==Wiimote::NUNCHUK)
			{
			for(int i=0;i<2;++i)
				{
				std::cout.setf(std::ios::fixed);
				std::cout.width(8);
				std::cout.precision(3);
				std::cout<<wiimote.getJoystickValue(i)<<" ";
				}
			}
		
		int numAccelerometers=wiimote.getExtensionDevice()==Wiimote::NUNCHUK?6:3;
		for(int i=0;i<numAccelerometers;++i)
			{
			std::cout.width(8);
			std::cout.precision(3);
			std::cout<<wiimote.getAccelerometerValue(i)<<" ";
			}
		std::cout<<std::flush;
		}
	
	return 0;
	}
