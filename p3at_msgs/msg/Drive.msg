# This message represents a low-level motor command to Pioneer.

# Command units dependent on the value of this field
int8 MODE_VELOCITY=0   # velocity command (rad/s of wheels)
int8 MODE_NONE=-1      # no control, commanded values ignored.
int8 mode

# Units as above, +ve direction propels chassis forward.
int8 LEFT=0
int8 RIGHT=1
float32[2] drivers
