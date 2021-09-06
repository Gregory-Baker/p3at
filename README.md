# Pioneer 3AT (P3AT) ROS Package

### Mission: 
- Revive an old Pioneer 3AT (4 wheel, differential drive) robot with a new microcontroller (Teensy 3.2), and onboard computer (currently Jetson Nano 4GB, previously Raspberry Pi 4, possibly Jetson Xavier NX next if I can get my hands on one).
- I plan to use the robot for my research in mixed reality (MR) teleoperation of mobile robots, but will aim to make it a general purpose platform 

### Current Setup:
![Adapted Pioneer 3AT Image](./p3at_resources/Pioneer3AT_adapted-min.png "Adapted Pioneer 3AT")
- Jetson Nano running Jetpack 4.5.
- Teensy 3.2 microcontroller to interface with the existing P3AT motor control board (which seems to work fine), running a custom sketch that can be found in [p3at_embedded folder](./p3at_embedded/p3at_teensy). 
- RPLidar A2M8 2D lidar scanner.

Specific to my requirements:
- Sparkfun Auto pHat on the Jetson Nano to control the servo pan-tilt mount for MR teleop.
- ZED Camera mounted on a servo pan-tilt module that tracks the users head in VR

### Implementation:
Code adapted from a variety of sources, but some main ones are:
- Clearpath [jackal](https://github.com/jackal/jackal), [jackal_robot](https://github.com/jackal/jackal_robot) and [husky](https://github.com/husky/husky) repos.
- Eborghi10's [my_ROS_mobile_Robot](https://github.com/eborghi10/my_ROS_mobile_robot) repo.

I am very grateful to the people that developed these and other fantastic open source resources. The world of robotics is undoubtedly a better place with you in it ;).

### My Method and Disclaimer:
I am relatively new to ROS, and would consider myself a 'functional' programmer, rather than a 'meticulous' one (i.e. I will usually move-on from a bit of code once it works sufficiently well, rather than taking time to make it perfect and/or generic). I have taken the learning-by-doing approach during this project. I therefore can't guarantee the efficiency or efficacy of any of my code. If you have any suggestions for improvements, then please drop them in the issues section of this repo or request a merge if you've implemented something extra yourself. If you have any other questions or ideas, then please feel free to email me at gregory.baker@brl.ac.uk.

### Other Useful Resources:
#### P3AT Docs:
https://www.inf.ufrgs.br/~prestes/Courses/Robotics/manual_pioneer.pdf
- Motor Control Board (MCB) pinout (p. 55) and diagram (p. 57) were particularly useful.

#### Understanding PID better:
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- When I first setup the PID control loop for velocity control of my wheels, the tuning was terrible (started with Kp = 1, Ki = 1, Kd = 0.01) - I hadn't tuned a PID controller before and didn't know where to begin with hand-tuning. But after maybe 30 mins - 1 hour of just playing around with the numbers I arrived at one that worked pretty well. It was actually pure integral control with a gain of Ki = 100. That was a good day... 

#### Communication between Onboard Computer and Teensy: 
http://wiki.ros.org/rosserial

#### Understanding ros_control better: 
https://fjp.at/posts/ros/ros-control/ and https://fjp.at/projects/diffbot/ros-packages/control/
- I'm much more familiar with C# and Python than I am with C++. It was therfore a bit of a shock when I realised that the interface between the hardware (which I'd just about struggled through with the help of the santized Arduino IDE) and the diff_drive_controller (that I knew I probably needed to use) was all in C++. ros_control is an immensely useful tool and I know it has saved me a lot of pain in the long run, but boy-oh-boy was it a nightmare to get my head around. The above links are a good starting point. The 'Diffbot' project actually has a lot of overlap with mine as well. Once I'd eventually got my head around some amount of ros_control I decided that the best course of action was to restructure my microcontroller code so I could copy the [jackal implemntation](https://github.com/jackal/jackal_robot/tree/melodic-devel/jackal_base) as closely as possible. This involved defining ros message types that look remarkably similar to [jackal_msgs](https://github.com/jackal/jackal/tree/melodic-devel/jackal_msgs) and loading the message defs to the Arduino IDE using rosserial_arduino (see [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)).

#### Robot Localization: 
https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- A good starting point for sensor fusion with the robot_localization package. I don't 100% understand what's going on under the hood with this package (and barely enough to configure it properly to be honest) but it seems to work to some extent. I am currently fusing odometry information from the diff_drive_controller (part of the ros_control suite) with the ICM20948 IMU on the Auto pHAT, but ignoring the magnetometer readings as they seem to mess things up (even though I've attempted hard-iron correction) plus I don't really need to align to the 'world' N-E-S-W coordinates at the moment because my use case is indoor navigation.

### Potential Issues You Might Face When Replicating:
1) This is currently a WIP, so I would be impressed if it worked first time on another system. I do have plans to do a fresh install once I've got sufficiently far with the core functionality and I'll update this README with a step-by-step.
