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
const int GL_CONTACT = 16;
const int GR_CONTACT = 18;
const int IR_INNER = 19;
const int IR_OUTER = 17; // Not working

// Used to reverse motor directions if neccessary
const int offsetA = 1;
const int offsetB = 1;


class Gripper {

  private:
    Motor liftMotor;
    Motor gripMotor;
    int speed_;

    bool gripperFullyOpen_;
    bool liftLimit_;
    bool liftLimitTop_;
    bool liftLimitBottom_;
    bool leftPaddleContact_;
    bool rightPaddleContact_;
    bool innerIR_;
    bool outerIR_;

    initPins();
    readPins();


  public:
    Gripper(int);
    Open();
    Close();
    Up();
    Down();
};

Gripper::Gripper(int speed = 155) :
  speed_(speed);
{

  Gripper::initPins();
  Gripper::readPins();

  // Need to change this to handle unknown state
  // when liftLimit switch is depressed at start
  liftLimitTop_ = false;
  liftLimitBottom_ = false;
}

Gripper::initPins() {
  liftMotor = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
  gripMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

  pinMode(G_OPEN, INPUT_PULLUP);
  pinMode(L_LIMIT, INPUT_PULLUP);
  pinMode(GL_CONTACT, INPUT_PULLUP);
  pinMode(GR_CONTACT, INPUT_PULLUP);
  pinMode(IR_INNER, INPUT);
  pinMode(IR_OUTER, INPUT);
}

Gripper::readPins() {
  gripperFullyOpen_ = !digitalRead(G_OPEN)
  liftLimit_ = !digitalRead(L_LIMIT)
  leftPaddleContact_ = !digitalRead(GL_CONTACT)
  rightPaddleContact_ = !digitalRead(GR_CONTACT)
  innerIR_ = digitalRead(IR_INNER)
  outerIR_ = digitalRead(IR_OUTER)
}

Gripper::Open() {
  // Don't continue to drive if fully open
  if(!gripperFullyOpen_) {
    gripMotor.drive(speed_);
  }
}

Gripper::Close() {
  // Don't continue to close gripper if both paddles in contact
  if(!leftPaddleContact || !rightPaddleContact) {
    gripMotor.drive(-speed_)
  }
}

Gripper::Up() {
  if (!liftLimit) {
    liftLimitTop_ = false
    liftLimitBottom_ = false;
  }
  if(!liftLimitTop_)
  {
    liftMotor.drive(speed_);
    bool liftLimitPost_ = digitalRead(L_LIMIT);
    if(liftLimitPost_ == true) {
      liftLimitTop_ = true;
    }
  }
}

Gripper::Down() {
  if (!liftLimit) {
    liftLimitTop_ = false
    liftLimitBottom_ = false;
  }
  if(!liftLimitBottom_)
  {
    liftMotor.drive(-speed_);
    bool liftLimitPost_ = digitalRead(L_LIMIT);
    if(liftLimitPost_ == true) {
      liftLimitBottom_ = true;
    }
  }
}
