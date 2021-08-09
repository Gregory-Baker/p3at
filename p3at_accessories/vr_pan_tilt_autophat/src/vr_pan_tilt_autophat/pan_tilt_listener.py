#!/usr/bin/env python

# Import required Python code.
import rospy
import pi_servo_hat
from geometry_msgs.msg import Vector3

pan_servo_channel = 0
pan_zero_angle = 52
pan_multiplier = 1
pan_min = -15
pan_max = 130

tilt_servo_channel = 1
tilt_zero_angle = 80
tilt_multiplier = 1.0


class Servo(object):
    
    def __init__(self, servo_hat, channel, zero_angle, multiplier=1.0, min = None, max = None):
        
        self.servo_hat = servo_hat
        self.channel = channel
        self.zero_angle = zero_angle
        self.servo = self.setup_servo()
        self.multiplier = multiplier
        self.min = min
        self.max = max

    def setup_servo(self):
        self.servo_hat.move_servo_position(self.channel, self.zero_angle)
    
    def set_angle(self, angle):
        servo_angle = self.limit_angle(self.multiplier * angle + self.zero_angle)
        self.servo_hat.move_servo_position(self.channel, servo_angle)

    def limit_angle(self, angle):
        if (self.min != None and angle < self.min):
            angle = self. min
        elif (self.max != None and angle > self.max):
            angle = self.max
        return angle 


class ServoHat(object):

    def __init__(self):
        self.servo_hat = pi_servo_hat.PiServoHat()
        self.servo_hat.restart()
        
        self.pan_servo = Servo(self.servo_hat, pan_servo_channel, pan_zero_angle, pan_multiplier, pan_min, pan_max)
        self.tilt_servo = Servo(self.servo_hat, tilt_servo_channel, tilt_zero_angle, tilt_multiplier)

    def stop(self):
        self.servo_hat.restart()


def callback(data):
    yaw = data.y
    pitch = data.x
    if (yaw > 180):
        yaw = -(360-yaw)
    if (pitch > 180):
        pitch = -(360-pitch)

    servo_hat.pan_servo.set_angle(yaw)
    servo_hat.tilt_servo.set_angle(pitch)

def listener():
    topic = rospy.get_param('~topic', 'head_rot')
    rospy.Subscriber(topic, Vector3, callback)
    rospy.spin()

def main():
    servo_hat = ServoHat()
    rospy.init_node('pan_tilt_listener')
    listener()
    






