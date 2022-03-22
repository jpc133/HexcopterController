#if defined(ARDUINO_ARC32_TOOLS)
  #define BOARD  101
#else
  #define BOARD  UNO
#endif

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#if BOARD == 101
  #include <CurieIMU.h>
  #include <CurieBLE.h>
  #include "CurieTimerOne.h"
#endif
#if BOARD == UNO
  #include <Arduino_LSM6DS3.h>
#endif
#include <MadgwickAHRS.h>
#include "HexMotor.h"
#include <math.h>
    
//Radio Pins
byte THROTTLE_IN_PIN = 10;
byte ROLL_IN_PIN = 8;
byte PITCH_IN_PIN = 9;
byte YAW_IN_PIN = 11;

byte ACC_IN_PIN = 12;

//PWM Driver Variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define PWM_MIN  205.0 // This is the 'minimum' pulse length count (out of 4096)
#define PWM_MAX  389.0 // This is the 'maximum' pulse length count (out of 4096)
#define PWM_USMIN  1100 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define PWM_USMAX  1900 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
//(1000000 / 50) = 20000   us / freq = us per cycle
//20000 / 4096 = 4.8828125 (divide microseconds by this to get 'pulse length count'
#define ESC_FREQ 50 // ESCs typically run at ~50 Hz updates (just like servos)

//Define Motor array
#define MOTORS_AVAILABLE 6 //This only works with Hexcopters with 60 degrees between arms with an arm straight forward and back
HexMotor motors[MOTORS_AVAILABLE] = {};
/*
See diagram below of motor layout
       5  L  6
        \   /
BACK  4 -BBB- 1  FRONT
        /   \
       3  R  2
Motor Positions: ([+/-]sin(60),[+/-]sin(60)) and also (0,1) and (0,-1)
1 - (0,1)
3 - (0.866,0.5)
3 - (0.866,-0.5)
4 - (0,-1)
5 - (-0.866,-0.5)
6 - (-0.866,0.5)
*/

//60Deg × π/180 = 1.047Rad
double positive_Motor_X_Offset = sin(60 * (PI / 180)); //Approx 0.866
double positive_Motor_Y_Offset = 0.5; //0.5 = cos(60deg)
 
//Radio Values
int pwm_throttle;
int pwm_roll;
int pwm_pitch;
int pwm_yaw;

int pwm_acc;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  Serial.begin(9600);

  motors[0] = {1, 1, false, 0.0,1.0};
  motors[1] = {2, 2, true, positive_Motor_X_Offset,positive_Motor_Y_Offset};
  motors[2] = {3, 3, false, positive_Motor_X_Offset,-positive_Motor_Y_Offset};
  motors[3] = {4, 4, true, 0.0,-1.0};
  motors[4] = {5, 5, false, -positive_Motor_X_Offset,-positive_Motor_Y_Offset};
  motors[5] = {6, 6, true, -positive_Motor_X_Offset,positive_Motor_Y_Offset};

 #if BOARD == 101
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);
 #else
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
 #endif

  filter.begin(25);


  pwm.begin();

    //The below message is taken from the sample project for controlling the Adafruit 1411 shield
    /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(25000000); //Lol I don't have an oscilliscope so this it is!
  pwm.setPWMFreq(ESC_FREQ);  // Analog servos run at ~50 Hz updates

  for(uint16_t i = 1; i <= MOTORS_AVAILABLE; i++) {
    pwm.setPWM(i, 0, PWM_MAX);
  }
  
  delay((1000/50)*5);//Delay enough time to allow the ESC to know this is the max value
  
  for(uint16_t i = 1; i <= MOTORS_AVAILABLE; i++) {
    pwm.setPWM(i, 0, PWM_MIN);
  }
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;

 // read raw data from IMU
 #if BOARD == 101
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);
 #else
  if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
  }
  if (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(gx, gy, gz);
  }
 #endif

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("{orientation: [");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print("], controlValues: [");

  //Timeout on pulseIn, microseconds
  #define PULSE_IN_TIMEOUT 25000

  //Throttle
  pwm_throttle = pulseIn(THROTTLE_IN_PIN, HIGH, PULSE_IN_TIMEOUT);
  double pwm_throttle_converted = ((pwm_throttle - 1069.0)/850.0);
  //Serial.println(pwm_throttle_converted);
  if(pwm_throttle < 900 || (pwm_throttle_converted > 0.01 && pwm_throttle_converted < 0.01)){
    pwm_throttle_converted = 0;
  }
  Serial.print(pwm_throttle_converted);
  Serial.print(",");

  //Roll
  pwm_roll = pulseIn(ROLL_IN_PIN, HIGH, PULSE_IN_TIMEOUT);
  double pwm_roll_converted = (((pwm_roll - 1069.0)/850.0) * 2)-1;
  //Serial.println(pwm_roll_converted);
  if(pwm_roll < 900 || (pwm_roll_converted > 0.01 && pwm_roll_converted < 0.01)){
    pwm_roll_converted = 0;
  }
  Serial.print(pwm_roll_converted);
  Serial.print(",");

  //Pitch
  pwm_pitch = pulseIn(PITCH_IN_PIN, HIGH, PULSE_IN_TIMEOUT);
  double pwm_pitch_converted = (((pwm_pitch - 1069.0)/850.0) * 2)-1;
  //Serial.println(pwm_pitch_converted);
  if(pwm_pitch < 900 || (pwm_pitch_converted > 0.01 && pwm_pitch_converted < 0.01)){
    pwm_pitch_converted = 0;
  }
  Serial.print(pwm_pitch_converted);
  Serial.print(",");

  //Yaw
  pwm_yaw = pulseIn(YAW_IN_PIN, HIGH, PULSE_IN_TIMEOUT);
  double pwm_yaw_converted = (((pwm_yaw - 1069.0)/850.0) * 2)-1;
  if(pwm_yaw < 900 || (pwm_yaw_converted > 0.01 && pwm_yaw_converted < 0.01)){
    pwm_yaw_converted = 0;
  }
  Serial.print(pwm_yaw_converted);
  Serial.print("]");

  pwm_acc = pulseIn(ACC_IN_PIN, HIGH, PULSE_IN_TIMEOUT);
  Serial.print(", acc: ");
  Serial.print(pwm_acc);
  Serial.print("}");
  
  double motorSpeeds[MOTORS_AVAILABLE] = {};
  double maxSpeed = 0;
  for(uint16_t i = 1; i <= MOTORS_AVAILABLE; i++) {    
    motorSpeeds[i-1] = motors[i-1].CalculateSpeed(pwm_throttle_converted, pwm_pitch_converted, pwm_roll_converted, pwm_yaw_converted);
    if(motorSpeeds[i-1] > maxSpeed){
      maxSpeed = motorSpeeds[i-1];
    }
  }
  
   for(uint16_t i = 1; i <= MOTORS_AVAILABLE; i++) {  
    if(pwm_throttle_converted > 0.01){
      motorSpeeds[i-1] = precision_map(motorSpeeds[i-1],0.0,maxSpeed,0,pwm_throttle_converted);
    }
    else{
      motorSpeeds[i-1] = 0;
    }
    //Serial.print(precision_map(motorSpeeds[i-1],0.0,1.0,PWM_MIN,PWM_MAX));
    //Serial.print("    ");  
    pwm.setPWM(i, 0, precision_map(motorSpeeds[i-1],0.0,1.0,PWM_MIN,PWM_MAX));
  }
  Serial.println("");
  
}

//precision_map is a clone of the standard lib map function which uses doubles instead of longs
//to enable using the logic for mapping values 0-1 to a different range (see mapping motor speeds)
double precision_map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
 
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
 float g;

 #if BOARD == 101
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  g = (gRaw * 250.0) / 32768.0;
 #else
  // since we are using 2000 degrees/seconds range
  // -2000 maps to a raw value of -32768
  // +2000 maps to a raw value of 32767
  g = (gRaw * 2000.0) / 32768.0;
 #endif

  return g;
}
