//Realeased under really Creative Commons! 
//This just a basic demo code...
//Reads and pulses the 8 channels... 
//By Jordi Munoz
#include <avr/interrupt.h>

volatile unsigned int Start_Pulse =0;
volatile unsigned int Stop_Pulse =0;
volatile unsigned int Pulse_Width =0;

volatile int Test=0;
volatile int Test2=0;
volatile int Temp=0;
volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;

long timer=0;
long timer2=0;

void setup()
{
  Init_PWM1(); //OUT2&3 
  Init_PWM3(); //OUT6&7
  Init_PWM5(); //OUT0&1
  Init_PPM_PWM4(); //OUT4&5
  Serial.begin(57600);

}
void loop()
{
       if((millis()- timer2) >= 20)
      {
        timer2=millis();
        if(All_PWM >2100)
        All_PWM=900;
        else
        All_PWM+=20;
      }
     OutputCh(0, All_PWM);
     OutputCh(1, All_PWM);
     OutputCh(2, All_PWM);
     OutputCh(3, All_PWM);
     OutputCh(4, All_PWM);
     OutputCh(5, All_PWM);
     OutputCh(6, All_PWM);
     OutputCh(7, All_PWM);     
  
  //Printing all values. 
  if((millis()- timer) >= 250)
  {
    timer=millis();
    Serial.print("Good.\n");
  }
   delay(20);
}


