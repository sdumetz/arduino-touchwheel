#include <Wire.h>
#include <Joystick.h>

#include "Filter.h"
#include "MPR121.h"
#include <Coordinates.h>


// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();
//ANGULAR_MULTIPLIER is a multiplication over the system's angular speed in rd/ms. You'll want 500/PI to have the system's rpm.
#define ANGULAR_MULTIPLIER 8000

#define THRESHOLD 3
#define SENSOR_COUNT 12

Filter filter = Filter();
float lastAngle;
float filteredRadius = 0;
int data[SENSOR_COUNT ];
Coordinates position = Coordinates();

//Prevent the loop from sending too much infos
unsigned long start = 0;

void setup() {

  //Serial.begin(9600);
  Joystick.begin(true);
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    //Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  //Serial.println("MPR121 found!");

  delay(10);
}
float angleDiff(float a, float b){
  if(abs(b-a) < PI ){
    return b-a;
  }else if(a <b) {
    return 2*PI-b+a;
  }else{
    return 2*PI+b-a;
  }
}

void loop() {
  int i;
  unsigned long end;
  int diff = 0;
  float x = 0,y = 0;
  float newAngle;
  int m;
  //linear decrease
  unsigned long loopStart = millis();
  for(i=0;i<SENSOR_COUNT ;i++){
    m = cap.baselineData(i) -cap.filteredData(i);
    if(m<0) m = 0;
    position.fromPolar(
      m, PI*2*i/SENSOR_COUNT
    );
    //Add them
    x += position.getX();
    y += position.getY();
  }
  //calculate ponderated position
  position.fromCartesian(x,y);
  /* Calculate running time */
  end = millis();
  filteredRadius = filteredRadius*0.5 + position.getR()*0.5;
  if(THRESHOLD < filteredRadius){
    //Angle is negative only if we don't detect any touch
    //We update angular speed only if THRESHOLD < R
    //AND a touch was detected on previous tick
    if(lastAngle != -1){
      diff = filter.update(
        (int)((angleDiff(lastAngle,position.getAngle())*ANGULAR_MULTIPLIER)/(end-loopStart)),
        (int)position.getR()
      );
    }else{
      diff = filter.update(0, 0 );
    }
    lastAngle = position.getAngle();
  }else{
    diff = filter.update(0, 0 );
    if( lastAngle != -1){
      lastAngle = -1;
      //Serial.println("Untouched");
    }
  }
  /*
  Serial.print( position.getR());
  Serial.print("\t");
  Serial.print( position.getAngle());
  Serial.print("\t");
  Serial.print(diff);
  Serial.print("\n");
  //*/
  delay(4);
  //at least 50ms since last update
  /* OUTPUT */
  Joystick.setXAxis(diff);

  //Serial.print((int)(filteredMean*200/(millis()-start)));

  //*/
  start = millis();
}
