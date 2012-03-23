// For the following sonar range finder:
// http://www.maxbotix.com/documents/MB1020_Datasheet.pdf
// Reads analog voltage (AN) and prints the distance in inches

int raw = 0;
float inches = 0;

void setup()
{
  pinMode(6,INPUT);
  Serial.begin(9600);
}

void loop()
{
  raw = analogRead(6);    // read the input pin
  //Serial.print(raw);             // debug value
  Serial.print(" Distance (in): ");
  inches = round(raw * 5 / 0.0098 / 1024);
  Serial.println(inches);
}
