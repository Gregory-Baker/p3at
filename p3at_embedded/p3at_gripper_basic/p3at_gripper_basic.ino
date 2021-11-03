/******************************************************************************
Pioneer Gripper Test Script
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker

******************************************************************************/

#include <ros.h>
#include "Gripper.h"
#include <p3at_gripper/Command.h>

p3at_gripper::Command cmd;

Gripper gripper;

ros::NodeHandle nh;

unsigned long cmdTime;

void messageCb( const p3at_gripper::Command &command_msg){
  cmdTime = millis();
  gripper.setCommand(command_msg.lift_command, command_msg.gripper_command);
}

ros::Subscriber<p3at_gripper::Command> sub("gripper_command", &messageCb );

void setup()
{
  cmdTime = millis();
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();

  unsigned long timeSinceCmd = millis() - cmdTime;
  if (timeSinceCmd > 500) {
    gripper.Stop(); 
  }
  gripper.actionCommand();
  
}
