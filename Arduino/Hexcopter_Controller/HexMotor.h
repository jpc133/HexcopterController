#pragma once
/*
  Motor.h
  Created by Jonathan Clarke, June 25th 2020
*/
#define HexMotor_h

#include "Arduino.h"

class HexMotor
{
  public:
    HexMotor();
    HexMotor(int pin, int motorNumber, bool ccw, double offsetX, double offsetY);
    double CalculateSpeed(double throttle, double pitch, double roll, double yaw);
    double Offset[2];
    bool CCW;
    bool CW;
    int MotorNumber;
    int Pin;
  private:
};
