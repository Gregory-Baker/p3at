/******************************************************************************
Pioneer Gripper Test Script
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker

******************************************************************************/

#include <ros.h>
#include "Gripper.h"
#include <p3at_gripper/Command.h>
#include <p3at_gripper/Feedback.h>

p3at_gripper::Command cmd;
p3at_gripper::Feedback feedback;
ros::Publisher pub("gripper_feedback", &feedback);

Gripper gripper;

ros::NodeHandle nh;

unsigned long cmdTime;
unsigned long pubTime;
unsigned long pubInterval = 100;

void messageCb( const p3at_gripper::Command &command_msg){
  cmdTime = millis();
  gripper.setCommand(command_msg.lift_command, command_msg.gripper_command);
}

ros::Subscriber<p3at_gripper::Command> sub("gripper_command", &messageCb );

bool readings[6] = {false, false, false, false, false, false};

void setup()
{
  cmdTime = millis();
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  nh.spinOnce();

  unsigned long timeSinceCmd = millis() - cmdTime;
  if (timeSinceCmd > 500) {
    gripper.Stop();
    feedback.gripper_command_received = false; 
  }
  else {
    feedback.gripper_command_received = true;
  }
  gripper.actionCommand();
  
  if ((millis() - pubTime) > pubInterval) {
    gripper.getRead(readings);
    feedback.gripper_open = readings[0];
    feedback.gripper_closed = readings[1];
    feedback.gripper_lift_up = readings[2];
    feedback.gripper_lift_down = readings[3];
    feedback.gripper_ir_outer = readings[4];
    feedback.gripper_ir_inner = readings[5];
    feedback.header.stamp = nh.now();
    pub.publish(&feedback);
    pubTime = millis();
  }
}
