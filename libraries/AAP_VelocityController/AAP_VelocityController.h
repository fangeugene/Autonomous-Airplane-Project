#ifndef AAP_VelocityController_h
#define AAP_VelocityController_h

class AAP_VelocityController
{
  public:
	AAP_VelocityController();
	float getOutput(float setpoint, float altitude, float kP, float kD, float lowerBound, float upperBound);
	//float lastOutput();
};
#endif
