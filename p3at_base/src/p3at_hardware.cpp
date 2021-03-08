/**
 *
 *  \file
 *  \brief      Class representing P3at hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 *  Modified by	Greg Baker <gregory.baker@brl.ac.uk> for use with Pioneer 3AT
 *  Repo: https://github.com/Gregory-Baker/p3at
 */

#include <boost/assign.hpp>
#include "p3at_base/p3at_hardware.h"

namespace p3at_base
{

P3atHardware::P3atHardware()
{
  ros::V_string joint_names = boost::assign::list_of("p3at_front_left_wheel_joint")
      ("p3at_front_right_wheel_joint")("p3at_back_left_wheel_joint")("p3at_back_right_wheel_joint");

  for (unsigned int i = 0; i < joint_names.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
        &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);

  feedback_sub_ = nh_.subscribe("feedback", 1, &P3atHardware::feedbackCallback, this);

  // Realtime publisher, initializes differently from regular ros::Publisher
  cmd_drive_pub_.init(nh_, "cmd_drive", 1);
}

/**
 * Populates the internal joint state struct from the most recent Feedback message
 * received from the MCU.
 *
 * Called from the controller thread.
 */
void P3atHardware::copyJointsFromHardware()
{
  boost::mutex::scoped_lock feedback_msg_lock(feedback_msg_mutex_, boost::try_to_lock);
  if (feedback_msg_ && feedback_msg_lock)
  {
    for (int i = 0; i < 4; i++)
    {
      joints_[i].position = feedback_msg_->drivers[i % 2].measured_travel;
      joints_[i].velocity = feedback_msg_->drivers[i % 2].measured_velocity;
      joints_[i].effort = 0;  // TODO(mikepurvis): determine this from amperage data.
    }
  }
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void P3atHardware::publishDriveFromController()
{
  if (cmd_drive_pub_.trylock())
  {
    cmd_drive_pub_.msg_.mode = p3at_msgs::Drive::MODE_VELOCITY;
    cmd_drive_pub_.msg_.drivers[p3at_msgs::Drive::LEFT] = joints_[0].velocity_command;
    cmd_drive_pub_.msg_.drivers[p3at_msgs::Drive::RIGHT] = joints_[1].velocity_command;
    cmd_drive_pub_.unlockAndPublish();
  }
}

void P3atHardware::feedbackCallback(const p3at_msgs::Feedback::ConstPtr& msg)
{
  // Update the feedback message pointer to point to the current message. Block
  // until the control thread is not using the lock.
  boost::mutex::scoped_lock lock(feedback_msg_mutex_);
  feedback_msg_ = msg;
}

}  // namespace p3at_base
