// For the following sonar range finder:
// http://www.maxbotix.com/documents/MB1020_Datasheet.pdf
// Reads analog voltage (AN) and prints the distance in inches

#include <AAP_Sonar.h>
AAP_Sonar sonar;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(round(sonar.getDistance()));
}