# This message represents high-frequency feedback from the MCU.

Header header

DriveFeedback[2] drivers

# Commanded control mode, use the TYPE_ constants from p3at_msgs/Drive.
int8 commanded_mode

# Actual control mode. This may differ from the commanded in cases where
# the motor enable is off, the motors are in over-current, etc.
int8 actual_mode
