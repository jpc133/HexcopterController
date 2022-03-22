/*
  Motor.h
  Created by Blake Clarke, June 25th 2020
*/
#include "Arduino.h"
#include "HexMotor.h"

HexMotor::HexMotor(){
}

HexMotor::HexMotor(int pin, int motorNumber, bool ccw, double offsetX, double offsetY){
    Offset[0] = offsetX;
    Offset[1] = offsetY;
    CCW = ccw;
    CW = !ccw;
    MotorNumber = motorNumber;
    Pin = pin;
}

double HexMotor::CalculateSpeed(double throttle, double pitch, double roll, double yaw){
    double motorSpeed;
    switch(MotorNumber){
    case 1:
      motorSpeed = ((1+pitch+yaw)/4)*throttle;
      break;
    case 2:
      motorSpeed =  ((1-roll+pitch-yaw)/4)*throttle;
      break;
    case 3:
      motorSpeed = ((1-roll-pitch+yaw)/4)*throttle;
      break;
    case 4:
      motorSpeed = ((1-pitch-yaw)/4)*throttle;
      break;
    case 5:
      motorSpeed = ((1+roll-pitch+yaw)/4)*throttle;
      break;
    case 6:
      motorSpeed = ((1+roll+pitch-yaw)/4)*throttle;
      break;
     default:
      return 0;
  }
  //This isn't absoluting the speed so much as making sure we don't try to run the motor backwards
  if(motorSpeed<0){
    return 0;
  }
  else{
    return motorSpeed;
  }
}
