	#ifndef AAP_IRCamera_h
#define AAP_IRCamera_h
#define byte uint8_t
#include <inttypes.h>

#include "../AP_Math/AP_Math.h"

class AAP_IRCamera
{
  public:
	AAP_IRCamera();

	//Assumes sources is an array of size 4
	//sources[i] is invalid if both sources[i].x and sources[i].y are equal to 1023
	void init();
	Vector2i getRawData();
	Vector3f getPosition();
  private:
  	int _IRsensorAddress;
	int _slaveAddress;
	byte _data_buf[16];
	void Write_2bytes(byte d1, byte d2);
};
#endif
