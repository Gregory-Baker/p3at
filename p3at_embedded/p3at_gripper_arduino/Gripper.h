/******************************************************************************
Pioneer Gripper Class
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker
Assumed motor config:
- Lift Up (+ve) and Down (-ve) on A_IN pins of TB6612 motor driver
- Gripper Out (+ve) and In (-ve) on B_IN pins of TB6612 motor driver

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

const int G_OPEN = 14;
const int L_LIMIT = 15;
const int L_LIMIT_TOP = 2;
const int GL_CONTACT = 16;
const int GR_CONTACT = 18;
const int IR_INNER = 19;
const int IR_OUTER = 17; // Not working

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

    bool gripperFullyOpen_;
    bool liftLimit_;
    bool liftLimitTop_;
    bool leftPaddleContact_;
    bool rightPaddleContact_;
    bool innerIR_;
    bool outerIR_;

  public:
    Gripper(int);
    void initPins();
    void readPins();
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
  Gripper::initPins();
  Gripper::readPins();
}

void Gripper::initPins() {
  pinMode(G_OPEN, INPUT_PULLUP);
  pinMode(L_LIMIT, INPUT_PULLUP);
  pinMode(L_LIMIT_TOP, INPUT);
  pinMode(GL_CONTACT, INPUT_PULLUP);
  pinMode(GR_CONTACT, INPUT_PULLUP);
  pinMode(IR_INNER, INPUT);
  pinMode(IR_OUTER, INPUT);
}

void Gripper::readPins() {
  gripperFullyOpen_ = !digitalRead(G_OPEN);
  liftLimit_ = !digitalRead(L_LIMIT);
  liftLimitTop_ = digitalRead(L_LIMIT_TOP);
  leftPaddleContact_ = !digitalRead(GL_CONTACT);
  rightPaddleContact_ = !digitalRead(GR_CONTACT);
  innerIR_ = digitalRead(IR_INNER);
  outerIR_ = digitalRead(IR_OUTER);
}

void Gripper::setCommand(int liftCommand, int gripCommand) {
  liftCommand_ = liftCommand;
  gripCommand_ = gripCommand;
}

void Gripper::actionCommand(){
  Gripper::readPins()
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
  // Don't continue to drive if fully open
  if(!gripperFullyOpen_) {
    gripMotor.drive(speed_);
  }
  else {
    gripMotor.brake();
  }
}

void Gripper::Close() {
  // Don't continue to close gripper if both paddles in contact
  if(!leftPaddleContact_ || !rightPaddleContact_) {
    gripMotor.drive(-speed_);
  }
  else {
    gripMotor.brake();
  }
}

void Gripper::Up() {
  if(!(liftLimit_ && liftLimitTop_))
  {
    liftMotor.drive(speed_);
  }
  else {
    liftMotor.brake();
  }
}

void Gripper::Down() {
  if(!(liftLimit_ && !liftLimitTop_))
  {
    liftMotor.drive(-speed_);
  }
  else {
    liftMotor.brake();
  }
}

void Gripper::Stop() {
  liftMotor.brake();
  gripMotor.brake();
}

void Gripper::StopLift() {
  liftMotor.brake();
}

void Gripper::StopGrip() {
  gripMotor.brake();
}
