#!/usr/bin/env python

"""
Node to pass joystick commands from /joy node to custom p3at gripper
"""

import rospy
from sensor_msgs.msg import Joy
from p3at_gripper.msg import Command

def joy_listener():

    # start node
    rospy.init_node("p3at_gripper_joy", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("joy", Joy, joy_cmd_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()

def joy_cmd_callback(data):

    # start publisher
    pub = rospy.Publisher("gripper_command", Command, queue_size=1)

    # create gripper command message
    cmd = Command()
    cmd.lift_command = int(data.axes[7])
    if (data.buttons[0] == 1):
        cmd.gripper_command = 1
    elif (data.buttons[1] == 1):
        cmd.gripper_command = -1
    else:
        cmd.gripper_command = 0

    # record values to log file and screen
    if not (cmd.gripper_command == 0 and cmd.lift_command == 0):
        rospy.loginfo("lift command: %d ; gripper command %d",cmd.lift_command, cmd.gripper_command)

    # publish gripper command
    pub.publish(cmd)


if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass


