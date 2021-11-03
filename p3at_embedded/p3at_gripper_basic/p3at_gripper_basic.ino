/******************************************************************************
Pioneer Gripper Test Script
for Bristol Robotics Lab custom P3AT project
Author: Greg Baker

******************************************************************************/

#include <ros.h>
#include "Gripper.h"
#include <p3at_gripper/Command.h>

p3at_gripper::Command cmd;

int gripper_motor_speed;

Gripper gripper;

ros::NodeHandle nh;

unsigned long cmdTime;

void messageCb( const p3at_gripper::Command &command_msg){
  cmdTime = millis();
  gripper.setCommand(command_msg.lift_command, command_msg.gripper_command);
}

ros::Subscriber<p3at_gripper::Command> sub("gripper_command", &messageCb );

void setSpeed() {
    
  if (! nh.getParam("~gripper_motor_speed", &gripper_motor_speed)) { 
    gripper_motor_speed = 200;
  } 
  else {
    nh.loginfo("Non-default gripper motor speed param set");
  }

  gripper_motor_speed = (gripper_motor_speed > 255) ? 255 : gripper_motor_speed;
  gripper_motor_speed = (gripper_motor_speed < 100) ? 100 : gripper_motor_speed;

  gripper.setMotorSpeed(gripper_motor_speed);

}

void setup()
{
  cmdTime = millis();
  nh.initNode();
  setSpeed();
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
