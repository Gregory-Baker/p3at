/*
    Script for the interfacing Teensy3.2 with a Pioneer3AT Motor Control Board

*/
#include "TeensyHW.h"
#include "DCMotor.h"
#include "OpticalEncoder.h"
#include <p3at_msgs/Drive.h>
#include <p3at_msgs/Feedback.h>

TeensyHW* teensy;

DCMotor* leftMotor;
DCMotor* rightMotor;

p3at_msgs::Feedback msg;
ros::Publisher pub("feedback", &msg);

int control_frequency;
float pid_gains[3];
long ticksPerRev = 99650;
int8_t command_mode =  0;
unsigned long previousCmdTime;

void commandCb(const p3at_msgs::Drive &drive_msg) {
  if (drive_msg.mode == 0) {
    command_mode = 0;
    leftMotor->SetTargetVelocity(drive_msg.drivers[0]);
    rightMotor->SetTargetVelocity(drive_msg.drivers[1]);
  }
  else {
    command_mode = -1;
  }
  previousCmdTime = millis();
}

ros::Subscriber<p3at_msgs::Drive> sub("cmd_drive", &commandCb);


IntervalTimer controlTimer;
boolean control_update_flag = false;

IntervalTimer slowTimer;
boolean slow_update_flag = false;
int slow_update_frequency = 1; //1Hz

unsigned long blinkTimer = millis();

void control_update_flag_on() {
  control_update_flag = true;
}

void slow_update_flag_on() {
  slow_update_flag = true;
}

void setup() {
  
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub);

  teensy = new TeensyHW();
  
  int control_frequency;
  if (! nh.getParam("control_frequency", &control_frequency)) { 
    //default value
    control_frequency = 50;
  }
  
  if (! nh.getParam("pid_gains", pid_gains, 3)) { 
    //default values
    pid_gains[0]= 0;
    pid_gains[1]= 100;
    pid_gains[2]= 0;
  }
  
  leftMotor = new DCMotor(LDIR, LPWM, LEFT
                           , new OpticalEncoder(LEA, LEB, LEFT, ticksPerRev)
                           , pid_gains[0], pid_gains[1], pid_gains[2]
                           , control_frequency);
                           
  rightMotor = new DCMotor(RDIR, RPWM, RIGHT
                           , new OpticalEncoder(REA, REB, RIGHT, ticksPerRev)
                           , pid_gains[0], pid_gains[1], pid_gains[2]
                           , control_frequency);
  
  int control_update_interval = 1000000/(control_frequency);   // Time between control updates (microseconds)
  controlTimer.begin(control_update_flag_on, control_update_interval);

  int slow_update_interval = 1000000/(slow_update_frequency);
  slowTimer.begin(slow_update_flag_on, slow_update_interval);

}

void loop() {

  nh.spinOnce();

  if (control_update_flag){

    // The following code disables PID if Pioneer is turned off, to prevent PID output balooning.
    float battVoltage = teensy->ReadBattery();

    int8_t actual_mode;

    unsigned long timeSinceCommand = millis() - previousCmdTime;
    
    if (timeSinceCommand < 500 && command_mode == 0 && battVoltage > 11.0) {      
      if (!leftMotor->pidOn) {
        teensy->LEDOn();
        leftMotor->PIDOn();
      }
      if (!rightMotor->pidOn) {
        rightMotor->PIDOn();
      }
      actual_mode = 0;
    }
    else {
      if (leftMotor->pidOn) {
        leftMotor->PIDOff();
        leftMotor->StopMotor();
      }
      if (rightMotor->pidOn) {
        rightMotor->PIDOff();
        rightMotor->StopMotor();
      }
      actual_mode = -1;
    }

    msg.header.stamp = nh.now();
    msg.drivers[p3at_msgs::Drive::LEFT] = leftMotor->Update();
    msg.drivers[p3at_msgs::Drive::RIGHT] = rightMotor->Update();
    msg.commanded_mode = command_mode;
    msg.actual_mode = actual_mode;

    pub.publish(&msg);

    control_update_flag = false;
  }

  if(slow_update_flag) {
    float battVoltage = teensy->ReadBattery();
    if (battVoltage < 11) {
      teensy->LEDToggle();
    }
    teensy->PublishBatteryVoltage();
    slow_update_flag = false;
  }
}
