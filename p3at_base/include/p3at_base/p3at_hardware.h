/**
 *
 *  \file
 *  \brief      Class representing P3AT hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 *  Modified by	Greg Baker <gregory.baker@brl.ac.uk> for use with Pioneer 3AT
 *  Repo: https://github.com/Gregory-Baker/p3at
 */

#ifndef P3AT_BASE_P3AT_HARDWARE_H
#define P3AT_BASE_P3AT_HARDWARE_H

#include "boost/thread.hpp"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "p3at_msgs/Drive.h"
#include "p3at_msgs/Feedback.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


namespace p3at_base
{

class P3atHardware : public hardware_interface::RobotHW
{
public:
  P3atHardware();
  void copyJointsFromHardware();
  void publishDriveFromController();

private:
  void feedbackCallback(const p3at_msgs::Feedback::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::Subscriber feedback_sub_;
  realtime_tools::RealtimePublisher<p3at_msgs::Drive> cmd_drive_pub_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // These are mutated on the controls thread only.
  struct Joint
  {
    double position;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0)
    {
    }
  }
  joints_[4];

  // This pointer is set from the ROS thread.
  p3at_msgs::Feedback::ConstPtr feedback_msg_;
  boost::mutex feedback_msg_mutex_;
};

}  // namespace p3at_base

#endif  // P3AT_BASE_P3AT_HARDWARE_H
