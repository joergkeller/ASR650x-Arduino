#include "Arduino.h"

#define INT_GPIO USER_KEY

#define timetillsleep 5000
static TimerEvent_t sleep;
uint8_t lowpower=1;

void onSleep()
{
  Serial.printf("Going into lowpower mode. Press user key to wake up\r\n");
  lowpower=1;
}
void onWakeUp()
{
  delay(10);
  if(digitalRead(INT_GPIO) == 0)
  {
	  Serial.printf("Woke up by GPIO, %d ms later into lowpower mode.\r\n",timetillsleep);
	  lowpower=0;
	  //timetillsleep ms later into lowpower mode;
	  TimerSetValue( &sleep, timetillsleep );
	  TimerStart( &sleep );
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(INT_GPIO,INPUT);
  attachInterrupt(INT_GPIO,onWakeUp,FALLING);
  TimerInit( &sleep, onSleep );
  Serial.printf("Going into lowpower mode. Press user key to wake up\r\n");
}

void loop() {
  if(lowpower){
    //note that lowPowerHandler() runs six times before the mcu goes into lowpower mode;
    lowPowerHandler();
  }
  // put your main code here, to run repeatedly:
}