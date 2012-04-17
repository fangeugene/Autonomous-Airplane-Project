// Reads I2C from the IR camera and prints the calculated position (x,y,z) of the camera in the beacon's coordinate frame

#include <FastSerial.h>
#include <AAP_IRCamera.h>
AAP_IRCamera IRCamera;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Vector3f pos = IRCamera.getPosition();
  Serial.printf("Camera position wrt beacon: %d %d %d\n", pos.x, pos.y, pos.z);
}
