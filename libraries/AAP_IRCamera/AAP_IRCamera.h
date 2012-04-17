#ifndef AAP_IRCamera_h
#define AAP_IRCamera_h

#include "../AP_Math/AP_Math.h"

class AAP_IRCamera
{
  public:
	AAP_IRCamera();

	//Assumes sources is an array of size 4
	//The IR source is invalid if sources[i].x and sources[i].y are equal to 1023? @Nick: revise this comment
	void getRawData(Vector2i sources[]);

	Vector3f getPosition();
};
#endif
