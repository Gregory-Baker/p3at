#!/usr/bin/env python

import time
import icm20948
from math import pi
from sensor_msgs.msg import Imu, MagneticField
import rospy

# TODO make I2C address of IMU a param or argument
imu = icm20948.ICM20948(icm20948.I2C_ADDR_ALT)
imu.set_accelerometer_full_scale(2)

def g_to_ms2(values):
	return [value*9.80665 for value in values]

def dps_to_radps(values):
	return [value*pi/180 for value in values]

def publish_imu_data(freq):
	rospy.init_node('icm20948', anonymous=True)
	pub_imu = rospy.Publisher('icm20948/data_raw', Imu, queue_size=10)
	pub_mag = rospy.Publisher('icm20948/mag', MagneticField, queue_size=10)
	rate = rospy.Rate(freq)

	while not rospy.is_shutdown():
		ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
		[ax, ay, az] = g_to_ms2([ax, ay, az])
		[gx, gy, gz] = dps_to_radps([gx, gy, gz])

		mag_x, mag_y, mag_z = imu.read_magnetometer_data()
		mag_msg = MagneticField()
		mag_msg.magnetic_field.x = mag_x
		mag_msg.magnetic_field.y = mag_y
		mag_msg.magnetic_field.z = mag_z
		mag_msg.header.stamp = rospy.Time.now()
		# TODO: Check if this frame_id is correct
		mag_msg.header.frame_id = ("mag")

		pub_mag.publish(mag_msg)
	
		imu_msg = Imu()
		# TODO: Input measurement covariances
		# imu_msg.orientation_covariance[0] = -1
		# imu_msg.angular_velocity_covariance[0] = -1
		# imu_msg.linear_acceleration_covariance[0] = -1

		imu_msg.linear_acceleration.x = ax
		imu_msg.linear_acceleration.y = ay
		imu_msg.linear_acceleration.z = az
		imu_msg.angular_velocity.x = gx
		imu_msg.angular_velocity.y = gy
		imu_msg.angular_velocity.z = gz
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = ("imu")

		pub_imu.publish(imu_msg)
		rate.sleep()

def main():
	rate = rospy.get_param('~rate', 50)
	rospy.loginfo("Started ICM20948 publisher")

	try:
		publish_imu_data(rate)
	except rospy.ROSInterruptException:
		pass

