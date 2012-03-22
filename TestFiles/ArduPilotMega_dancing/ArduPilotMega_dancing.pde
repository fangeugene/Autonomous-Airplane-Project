#include <avr/interrupt.h>

volatile unsigned int Start_Pulse =0;
volatile unsigned int Stop_Pulse =0;
volatile unsigned int Pulse_Width =0;

volatile int Test=0;
volatile int Test2=0;
volatile int Temp=0;
volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;

long timer=0;
long timer2=0;

int PWM_SEQ[4][8] = {
  {
    2000,2000,2000,2000,2000,2000,2000,2000    }
  , 
  {
    1000,1000,1000,1000,1000,1000,1000,1000    }
  ,
  {
    1000,1000,1000,1000,2000,2000,2000,2000    }
  ,
  {
    2000,2000,2000,2000,1000,1000,1000,1000    }
};

void setup()
{
  Init_PWM1(); //OUT2&3 
  Init_PWM3(); //OUT6&7
  Init_PWM5(); //OUT0&1
  Init_PPM_PWM4(); //OUT4&5
  Serial.begin(57600);

  OutputCh(0, 1200);
  OutputCh(1, 1200);
  OutputCh(2, 1200);
  OutputCh(3, 1200);
  OutputCh(4, 1200);
  OutputCh(5, 1200);
  OutputCh(6, 1200);
  OutputCh(7, 1200);

}
void loop()
{

  for(int w=0; w<=5; w++)
  {
    for(int y=0; y<2; y++)
    {
      for(int x=0; x<=7; x++)
      {
        OutputCh(x, PWM_SEQ[y][x]);
        delay(100);
      }     
    }
  }
  
/***************************************/  
  for(int w=0; w<=5; w++)
  {
    for(int y=2; y<4; y++)
    {
      for(int x=0; x<=7; x++)
      {
        OutputCh(x, PWM_SEQ[y][x]);
        delay(1);
      }     
      delay(300);
    }
  }

}





