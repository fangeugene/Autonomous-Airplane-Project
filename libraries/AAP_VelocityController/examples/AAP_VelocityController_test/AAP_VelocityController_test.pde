// For the following sonar range finder:
// http://www.maxbotix.com/documents/MB1020_Datasheet.pdf
// Reads analog voltage (AN) and prints the distance in inches

#include <AAP_VelocityController.h>
#include <AAP_Sonar.h>

AAP_VelocityController aapVC;
AAP_Sonar aapSonar;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  float altitude = aapSonar.getDistance() - 15;
  float setpoint = 100; // in centimeters
 
  Serial.print(altitude);
  Serial.print("               ");
  Serial.println(aapVC.getOutput(setpoint,altitude,0.001,0));
}