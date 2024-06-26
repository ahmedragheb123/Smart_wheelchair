/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/
#include <Arduino.h>
#include <mpu_encoder.h>
#include <encoder_real.h>
#include <Ultrasonic.h>

int direction1 = 0;
int direction2 = 0;
float rpm;

void setup() 
{
  Serial.begin(9600);
  intitialize_mpu_encoder();
  ultrasonic_init();
  
}

void loop()
{
  
  float rollkal = roll();
  float pitchkal = pitch();
  Serial.print(rollkal);
  Serial.print(":");
  Serial.print(pitchkal);
  float rollabs = abs(rollkal);
  float pitchabs = abs(pitchkal);
  Serial.println();
  rpm = rpmreq();

if (rollabs > pitchabs){
  
   
  // SETTING THE X DIRECTION RANGES OF THE HEADSET
  if(rollkal > 0  && rollkal < 90){            //FORWARD DIRECTION
  if(measureDistanceForward()){
  if(direction1 == -1 && rpm > 10){
      pid_controller(0,direction1,0,direction2);
  }
  else{
    if (rollkal > 15 && rollkal < 45){
      direction1 = 1;
      direction2 = -1;
      pid_controller(30,direction1,30,direction2);
    }
    else if(rollkal > 50 && rollkal < 90 ){
      direction1 = 1;
      direction2 = -1;
      pid_controller(60,direction1,60,direction2);
    }
    else if(rollkal < 10 && rollkal > -10){
     pid_controller(0,direction1,0,direction2);
    }
  }
  }
  else{
    pid_controller(0,direction1,0,direction2);
  }
  }

  else if(rollkal< 0  && rollkal > -90){         //BACKWARD DIRECTION
  if(measureDistanceReverse()){
  if(direction1 == 1 && rpm > 10){
      pid_controller(0,direction1,0,direction2);
  }
  else{
   if (rollkal < -15 && rollkal > -45 ){
      direction1 = -1;
      direction2 = 1;
      pid_controller(30,direction1,30,direction2);
    }
    else if(rollkal < -50 && rollkal  > -90 ){
      direction1 = -1;
      direction2 = 1;
      pid_controller(60,direction1,60,direction2);
    }
    else if(rollkal < 10 && rollkal> -10 ){
    pid_controller(0,direction1,0,direction2);
    }
  }
  }
  else{
    pid_controller(0,direction1,0,direction2);
  }
  }
}
else{
  if(measureDistanceForward()){
// SETTING THE Y DIRECTION RANGES OF THE HEADSET
  if(pitchkal < 0  && pitchkal > -70){               //RIGHT DIRECTION
    if (pitchkal < -15 && pitchkal > -45){
      direction1 = 1;
      direction2 = -1;
      pid_controller(20,direction1,50,direction2);
    }
    else if(pitchkal < -50 && pitchkal > -70 ){
      direction1 = 1;
      direction2 = -1;
      pid_controller(40,direction1,80,direction2);
    }
    else if(pitchkal < 10 && pitchkal > -10){
    pid_controller(0,direction1,0,direction2);
    }
  }
  
  else if(pitchkal > 0  && pitchkal < 70){           //LEFT DIRECTION
   if (pitchkal > 15 && pitchkal < 45 ){
      direction1 = 1;
      direction2 = -1;
      pid_controller(50,direction1,20,direction2);
    }
    else if(pitchkal > 50 && pitchkal  < 70 ){
      direction1 = 1;
      direction2 = -1;
      pid_controller(80,direction1,40,direction2);
    }
    else if(pitchkal < 10 && pitchkal> -10 ){
    pid_controller(0,direction1,0,direction2);
    }
  }
  }
   else{
    pid_controller(0,direction1,0,direction2);
  }
}
}