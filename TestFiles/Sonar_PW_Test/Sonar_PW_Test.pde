// For the following sonar range finder:
// http://www.maxbotix.com/documents/MB1020_Datasheet.pdf
// Reads pulse width (PW) and prints the distance in inches

int pin = 13;
int pulse = 0;
int inches = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  pulse = pulseIn(pin,HIGH);
  inches = pulse/147; 
  Serial.println(inches);

}
