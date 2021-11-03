/******************************************************************************
Pioneer Gripper Class
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker
Assumed motor config:
- Lift Up (+ve) and Down (-ve) on A_IN pins of TB6612 motor driver
- Gripper In (+ve) and Out (-ve) on B_IN pins of TB6612 motor driver
Requires:
Sparkfun_TB6612 library
******************************************************************************/

// Pioneer Gripper Class

#include <SparkFun_TB6612.h>

// Setup pin values
const int AIN1 = 5;
const int BIN1 = 7;
const int AIN2 = 4;
const int BIN2 = 8;
const int PWMA = 3;
const int PWMB = 9;
const int STBY = 6;

// Used to reverse motor directions if neccessary
const int offsetA = 1;
const int offsetB = 1;


class Gripper {

  private:
    Motor gripMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
    Motor liftMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
    int speed_;
    int liftCommand_;
    int gripCommand_;

  public:
    Gripper(int);
    void setCommand(int, int);
    void actionCommand();
    void Open();
    void Close();
    void Up();
    void Down();
    void Stop();
    void StopLift();
    void StopGrip();
};

Gripper::Gripper(int motorSpeed = 155) {
  speed_ = motorSpeed;
  liftCommand_ = 0;
  gripCommand_ = 0;
}

void Gripper::setCommand(int liftCommand, int gripCommand) {
  liftCommand_ = liftCommand;
  gripCommand_ = gripCommand;
}

void Gripper::actionCommand(){
  if (liftCommand_ == 1) {
    Gripper::Up();
  }
  else if(liftCommand_ == -1) {
    Gripper::Down();
  }
  else {
    Gripper::StopLift();
  }

  if (gripCommand_ == -1) {
    Gripper::Open();
  }
  else if (gripCommand_ == 1) {
    Gripper::Close();
  }
  else {
    Gripper::StopGrip();
  }
}

void Gripper::Open() {
  gripMotor.drive(speed_);
}

void Gripper::Close() {
  gripMotor.drive(-speed_);
}

void Gripper::Up() {
  liftMotor.drive(speed_);
}

void Gripper::Down() {
  liftMotor.drive(-speed_);
}

void Gripper::Stop() {
  liftCommand_ = 0;
  gripCommand_ = 0;
  liftMotor.brake();
  gripMotor.brake();
}

void Gripper::StopLift() {
  liftCommand_ = 0;
  liftMotor.brake();
}

void Gripper::StopGrip() {
  gripCommand_ = 0;
  gripMotor.brake();
}
