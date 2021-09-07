/******************************************************************************
Pioneer Gripper Test Script
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker

Assumed motor config:
Motor1 = Lift Up (+ve) and Down (-ve)
Motor2 = Gripper Out (+ve) and In (-ve)

Requires:
Sparkfun_TB6612 library

******************************************************************************/

// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>

const int AIN1 5
const int BIN1 7
const int AIN2 4
const int BIN2 8
const int PWMA 3
const int PWMB 9
const int STBY 6

const int G_OPEN 14
const int L_LIMIT 15
const int GL_CONTACT 16
const int GR_CONTACT 18
const int IR_INNER 19
const int IR_OUTER 17 // Not working



// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initialize motors.
Motor liftMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor gripperMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

void setup()
{
 //Nothing here
}


void loop()
{
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor1.drive(155,1000);
   motor1.brake();
   delay(1000);
//
//   motor2.drive(-155,1000);
//   motor2.brake();
//   delay(1000);
//
//   motor1.drive(-155,1000);
//   motor1.brake();
//   delay(1000);
//   
//   motor2.drive(+155,1000);
//   motor2.brake();
//   delay(1000);
   
}
