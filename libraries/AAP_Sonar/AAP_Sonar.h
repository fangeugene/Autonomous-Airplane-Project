#ifndef AAP_Sonar_h
#define AAP_Sonar_h

//#include <stdlib.h>
#define AAP_Sonar_MIN_DISTANCE 5 // in inches
#define AAP_Sonar_MAX_DISTANCE 250
#define AAP_Sonar_PIN = 6;

class AAP_Sonar
{
  public:
	AAP_Sonar();
	float getDistance();	
};
#endif
