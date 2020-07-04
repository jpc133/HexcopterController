/*
  Motor.h
  Created by Jonathan Clarke, June 25th 2020
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
  double nonAbsSpeed = _CalculateSpeed(throttle, pitch, roll, yaw);
  //This isn't absoluting the speed so much as making sure we don't try to run the motors backwards
  if(nonAbsSpeed<0){
    return 0;
  }
  else{
    return nonAbsSpeed;
  }
}


double HexMotor::_CalculateSpeed(double throttle, double pitch, double roll, double yaw){
  switch(MotorNumber){
    case 1:
      return ((1+pitch+yaw)/4)*throttle;
      break;
    case 2:
      return ((1-roll+pitch-yaw)/4)*throttle;
      break;
    case 3:
      return ((1-roll-pitch+yaw)/4)*throttle;
      break;
    case 4:
      return ((1-pitch-yaw)/4)*throttle;
      break;
    case 5:
      return ((1+roll-pitch+yaw)/4)*throttle;
      break;
    case 6:
      return ((1+roll+pitch-yaw)/4)*throttle;
      break;
     default:
      return 0;
  }
}
