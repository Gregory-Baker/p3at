# Writing Sketch to Teensy 3.2

Our setup uses a Teensy 3.2 microcontroller and [rosserial](http://wiki.ros.org/rosserial), using custom ros messages, defined in p3at_msgs package [1]. The Teensy is sent left and right wheel angular velocities (rad/s) and enacts them using a PID controller [2], with the encoder ticks (MCB pins 6,8,10,12) providing feedback.

## Upload Process

I use Arduino IDE with the Teensyduino Add-On to install the sketch to the Teensy board. 

It's advisable to install these components (Arduino IDE and Teensyduino) on the P3AT's onboard computer for 2 reasons. 1) The computer needs to have ROS and our custom message headers (p3at_msgs) already built. 2) If we need to tweak the sketch and re-upload, we can do so without accessing or unplugging the Teensy.


On your Pioneer's onboard computer:
<ol>
  <li>Follow the relevant <a href="https://www.pjrc.com/teensy/td_download.html">installation instructions for Teensyduino</a>. </li>
  <li>Install rosserial_arduino ros package, using `sudo apt install ros-melodic-rosserial-arduino`</li>
  <li>Make sure custom p3at_msgs have been generated - p3at_msgs package must be within catkin workspace, and then `catkin_make`</li>
  <li>Generate ros library for embedded platform - there's a bit of confusion about whether to use `rosserial_client` or `rosserial_arduino` for this step as both have `make_libraries` scripts; I've only ever succeeded with the latter. In my case, I use 'rosrun rosserial_arduino make_libraries ~/Arduino/libraries'. Make sure to point the final argument to where your Arduino installs it's libraries. [3]</li>
  <li>Now install any additional library dependencies using the Arduino IDE - You will need PID_v2 (Max_Ignatenko) and Encoder (Paul Stoffregen) libraries.</li>
  <li>Set the pin mapping in TeensyHW.h to the correct values (see section below) and tweak variable values in the sketch where necessary, e.g. ticksPerRev (number of encoder ticks per wheel revolution) is set to Pioneer 3 AT value, but may be different for other Pioneer models. </li>
  <li>Now compile and upload script to Teensy</li>
</ol>

Connect Teensy via USB to jetson and everything should work. You can test the setup quickly using `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200` (assuming Teensy has opened on port /dev/ttyACM0). You should see a few publishers and subscribers (e.g. cmd_drive, feedback and status) being registered. 



## Pin Mappings

Pin mappings are set in the TeensyHW.h header file within the p3at_teensy sketch. Our circuit is shown in the picture below 

<img src="https://github.com/Gregory-Baker/p3at/blob/main/p3at_resources/p3at_teensy_mcb_hookup_v2.png" alt="Teensy <-> Pioneer MCB Circuit" width="600"/>

| Description       | Pioneer MCB Pin | Teensy Pin  | Pin Name    | Notes         |
| ----              | ----            | ----        | ----        | ----          |
| Left Motor PWM    | 1               | 10          | LPWM        | 10K Pull-Down Resistor |
| Right Motor PWM   | 3               | 9           | RPWM        | 10K Pull-Down Resistor |
| Motors Enable     | 5               | 8           | MEN         | |
| E-Stop Detect     | 7               | 7           | ESTOP       | |
| | | |
| Left Motor Direction    | 2           | 14          | LDIR        | |
| Right Motor Direction   | 4           | 15          | RDIR        | |
| Left Encoder Channel A  | 6           | 16          | LEA         | |
| Right Encoder Channel A | 8           | 17          | REA         | |
| Right Encoder Channel B | 10          | 20          | REB         | |
| Left Encoder Channel B  | 12          | 21          | LEB         | |
| | | |
| Battery Voltage Analog  | 22          | A9          | VBAT        | 10K Resistor to Ground [4] |

## Notes:

[1] - As with the Clearpath Jackal, we use a rosserial baud rate of 115200 for the communication between the jetson board and the teensy.

[2] - We are in fact using pure integral control, because it performed the best during my initial hand-tuning.

[3] - If anyone knows how to get rosserial_client make_libraries working correctly, as per the [ros wiki tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Message) then let me know.

[4] - This is to divide the analog voltage from 0 - 5V range to 0 - 3V3 range that Teensy can read.
