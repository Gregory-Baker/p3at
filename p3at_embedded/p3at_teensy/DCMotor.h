#pragma once

#include <PID_v2.h>
#include "OpticalEncoder.h"
#include <p3at_msgs/DriveFeedback.h>

class DCMotor {
  
  private:
  
    const int DIR_pin, PWM_pin;
    const boolean wheelLR;
    OpticalEncoder* encoder;
    const double Kp, Ki, Kd;
    const int control_frequency;
    const int pwmFrequency;
    p3at_msgs::DriveFeedback msg;
    
    int motor_direction;
    double motor_pwm;
    double motor_velocity;
    double motor_speed;
    double motor_target_velocity;
    double motor_target_speed;
    double motor_angle;
    PID* motorPID;
  
    void initPins();
    void SetMotorDir(float);
    void SetMotorSpeed();
    void UpdateFeedbackMsg();
    void ReadEncoder();
  
  public:
  
    DCMotor(const int, const int, const boolean, OpticalEncoder*, double, double, double, const int);
    void UpdateControl();
    p3at_msgs::DriveFeedback Update();
    void PIDOn();
    void PIDOff();
    boolean pidOn;
    void SetTargetVelocity(float);
    void StopMotor();
  
};

DCMotor::DCMotor(const int DIR_pin, const int PWM_pin, const boolean wheelLR, OpticalEncoder* encoder, double Kp, double Ki, double Kd, const int control_frequency)
  : DIR_pin(DIR_pin)
  , PWM_pin(PWM_pin)
  , wheelLR(wheelLR)
  , encoder(encoder)
  , Kp(1)
  , Ki(0)
  , Kd(0)
  , control_frequency(control_frequency)
  , pwmFrequency(32000)
{
  DCMotor::initPins();
  StopMotor();
  motorPID = new PID(&motor_speed, &motor_pwm, &motor_target_speed, Kp, Ki, Kd, P_ON_M, DIRECT);
  motorPID->SetSampleTime(1000/control_frequency);
  PIDOn();
}

void DCMotor::initPins() {
  pinMode(DIR_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  analogWriteFrequency (PWM_pin, pwmFrequency);
}

void DCMotor::StopMotor() {
  SetTargetVelocity(0.0);
}

void DCMotor::SetTargetVelocity(float val) {
  motor_target_velocity = val;
  motor_target_speed = abs(val);
  SetMotorDir(val);
  if (val == 0 && !pidOn) {
    motor_pwm = 0;
  }
}

void DCMotor::SetMotorDir(float val) {
  if (val > 0) {
    motor_direction = 1;
    digitalWrite(DIR_pin, wheelLR);
  }
  else {
    motor_direction = -1;
    digitalWrite(DIR_pin, !wheelLR);
  }
}

void DCMotor::ReadEncoder(){
  motor_velocity = encoder->GetVelocity();
  motor_speed = abs(motor_velocity);
  motor_angle = encoder->GetAngle();
}


void DCMotor::SetMotorSpeed(){
  analogWrite(PWM_pin, motor_pwm);
}

void DCMotor::PIDOn() {
    motorPID->SetMode(AUTOMATIC);
    pidOn = true;
}

void DCMotor::PIDOff() {
    motorPID->SetMode(MANUAL);
    pidOn = false;
}

void DCMotor::UpdateFeedbackMsg(){
  msg.measured_velocity = motor_velocity;
  msg.measured_travel = motor_angle;
}

p3at_msgs::DriveFeedback DCMotor::Update(){
  UpdateControl();
  UpdateFeedbackMsg();
  return msg;
}

void DCMotor::UpdateControl() {
  ReadEncoder();
  motorPID->Compute();
  SetMotorSpeed();  
}
