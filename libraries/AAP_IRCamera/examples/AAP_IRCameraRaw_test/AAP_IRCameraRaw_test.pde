// Reads I2C from the IR camera and prints the x, y variables of each IR source

#include <FastSerial.h>
#include <AAP_IRCamera.h>
AAP_IRCamera IRCamera;

void setup()
{
  Serial.begin(9600);
  IRCamera.init();
}

void loop()
{
  Vector2i sources[4];
  IRCamera.getRawData(sources);
  Serial.printf("S0: %i %i   S1: %i %i   S2: %i %i   S3: %i %i\n", sources[0].x, sources[0].y,
                                                                   sources[1].x, sources[1].y,
                                                                   sources[2].x, sources[2].y,
                                                                   sources[3].x, sources[3].y);
}
