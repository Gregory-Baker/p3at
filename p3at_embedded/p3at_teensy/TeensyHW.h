#ifndef TEENSYHW_H
#define TEENSYHW_H

#define USE_USBCON
#include <ros.h>
#include <p3at_msgs/Status.h>

ros::NodeHandle nh;

const int LED = 13;           // LED pin

const int LPWM = 3;          // Left motor PWM
const int RPWM = 4;          // Right motor PWM

const int LDIR = 23;           // Left motor direction
const int RDIR = 22;           // Right motor direction

const int LEA = 21;            // Left encoder A
const int REA = 20;            // Right encoder A
const int REB = 19;            // Right encoder B
const int LEB = 18;            // Left encoder B

const int MEN = 10;           // Motor enable
const int ESTOP = 11;         // E-Stop detect input
const int VBAT = A1;          // Battery Voltage

/*  
 *   Note: VBAT pin is connected to Teensy3.2 ADC pin, but also has a 10k resistor to ground 
 *   Internal resistance of MCB is ~6.5k, achieving a voltage divider circuit that converts VBAT 0-5V to 0-3V3 range that the Teensy can read.
 *   The VBAT -> battery voltage conversion for our system has the following gradient and slope.
*/
const float vbat_m = 0.0165;  // Gradient of VBAT read -> battery voltage trendline
const float vbat_c = -0.386;  // y-intercept of VBAT read -> battery voltage trendline

class TeensyHW {
  
  private:
    void initPins();
    boolean ledState;
    p3at_msgs::Status msg;
    ros::Publisher pub;
    double battery_voltage;
    
  public:
    TeensyHW();
    void LEDOn();
    void LEDOff();
    void LEDToggle();
    float ReadBattery();
    void EnableMotors();
    void DisableMotors();
    void PublishBatteryVoltage();
  
};

TeensyHW::TeensyHW()
  : pub("status", &msg)
{
  initPins();
  EnableMotors();
  LEDOn();
  nh.advertise(pub);
}

void TeensyHW::initPins() {
  pinMode(LED, OUTPUT);                   // LED
  pinMode(MEN, OUTPUT);                   // Motor enable
  pinMode(VBAT, INPUT);                   // Battery voltage, analog input (10 bit resolution)
  pinMode(ESTOP, INPUT);                  // Detect ESTOP input
}

void TeensyHW::LEDOn() {
  ledState = HIGH;
  digitalWrite(LED, ledState);
}

void TeensyHW::LEDOff() {
  ledState = LOW;
  digitalWrite(LED, ledState);
}

void TeensyHW::LEDToggle() {
  ledState = !ledState;
  digitalWrite(LED, ledState);
}

float TeensyHW::ReadBattery() {
  return vbat_m*(float)analogRead(VBAT) + vbat_c;
}

void TeensyHW::EnableMotors() {
  digitalWrite(MEN, LOW);
}

void TeensyHW::DisableMotors() {
  digitalWrite(MEN, HIGH);
}

void TeensyHW::PublishBatteryVoltage(){
  // TODO: Add header to status message
  battery_voltage = ReadBattery();
  // msg.header.stamp = nh.now();
  msg.battery_voltage = battery_voltage;
  pub.publish(&msg);
}

#endif
