/******************************************************************************
Pioneer Gripper Test Script
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker

******************************************************************************/

#include "Gripper.h"
#include <ros.h>
#include <p3at_gripper/Command.h>

p3at_gripper::Command cmd;

Gripper gripper;

ros::NodeHandle nh;

void messageCb( const p3at_gripper::Command &command_msg){
  if (command_msg.lift_command == 1) {
    gripper.Up();
  }
  else if (command_msg.lift_command == -1) {
    gripper.Down();
  }
  if (command_msg.gripper_command == 1) {
    gripper.Open();
  }
  else if (command_msg.gripper_command == -1) {
    gripper.Close();
  }
}

ros::Subscriber<p3at_gripper::Command> sub("gripper_command", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  gripper.readPins();
  nh.spinOnce();
}
