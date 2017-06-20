#include "Filter.h"



Filter::Filter(){
  sensed_strength = 0; //Radius moving average
  speed = 0; //resulting speed
}

//Take newly measured angles, radius and time since last update.
//Return this->speed
int Filter::update (int d_angle,int radius){
  int confidence,dspeed,newSpeed;
  confidence = confidence*0.6;
  if (radius < 100){
    confidence += radius*0.4;
  }else{
    //A radius > 100 is expected to be unusually high
    confidence += 100*0.4;
  }
  if (confidence < 3){
    //Really low trust, skip update
    if(speed < -4){
      speed ++;
    }else if(4 < speed ){
      speed --;
    }else if(speed < 0 && -4 < speed ){
      speed --;
    }else if (0 <= speed && speed < 4){
      speed ++;
    }
    return speed;
  }
  // we calculate d²angle/dt² to apply a 2nd level filter
  dspeed = speed - d_angle;
  if(abs(dspeed) < 32){
    confidence = confidence*(40-abs(dspeed))/40;
  }else{
    confidence = confidence*0.2;
  }

  speed = (speed*(100-confidence)+d_angle*confidence)/100;

  return speed;
}

int Filter::getSpeed(){
  return speed;
}
