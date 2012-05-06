// Reads I2C from the IR camera and prints the x, y variables of each IR source

#include <FastSerial.h>
#include <AAP_IRCamera.h>
#include <I2C.h>

AAP_IRCamera IRCamera;
//const int resetPin = A0;

FastSerialPort0(Serial);

void setup()
{
  Serial.begin(9600);
  //pinMode(resetPin, OUTPUT);
  //analogWrite(resetPin, 168);
  IRCamera.init();
}

void loop()
{
  Vector2i sources[4];
  //IRCamera.getRawData(sources);
  /*Serial.printf("S0: %i %i   S1: %i %i   S2: %i %i   S3: %i %i\n", sources[0].x, sources[0].y,
                                                                   sources[1].x, sources[1].y,
                                                                   sources[2].x, sources[2].y,
                                                                   sources[3].x, sources[3].y);*/
  uint8_t intensity[4];
  IRCamera.getRawDataFull(sources, intensity);
  Serial.printf("S0: %i %i %u\t\t   S1: %i %i %u\t\t   S2: %i %i %u\t\t  S3: %i %i %u\n",  sources[0].x, sources[0].y, intensity[0],
                                                                               sources[1].x, sources[1].y, intensity[1],
                                                                               sources[2].x, sources[2].y, intensity[2],
                                                                               sources[3].x, sources[3].y, intensity[3]);
  /*Serial.printf("S0: %i %i   S1: %i %i   S2: %i %i   S3: %i %i\n", sources[0].x, sources[0].y,
                                                                   sources[1].x, sources[1].y,
                                                                   sources[2].x, sources[2].y,
                                                                   sources[3].x, sources[3].y);*/
 
}
