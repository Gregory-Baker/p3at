# Pioneer 3AT (P3AT) ROS Package

### Mission: 
- Revive an old Pioneer 3AT (4 wheel, differential drive) robot with a new microcontroller (Teensy 3.2), and onboard computer (Raspberry Pi 4B for now, plan to switch to Jetson Nano 4GB in the future).

### Current Setup:
![Adapted Pioneer 3AT Image](https://github.com/Gregory-Baker/p3at/blob/main/p3at_resources/P3AT_internal_atd2.png "Adapted Pioneer 3AT")
- I plan to add a RPLidar A2M8 and integrate the robot with the navigation stack next

### Implementation:
Code adapted from a variety of sources, but some main ones are:
- Clearpath [jackal](https://github.com/jackal/jackal), [jackal_robot](https://github.com/jackal/jackal_robot) and husky (https://github.com/husky/husky) repos.
- Eborghi10's [my_ROS_mobile_Robot](https://github.com/eborghi10/my_ROS_mobile_robot) repo.

I am very grateful to the people that developed these and other fantastic open source resources. The world of robotics is undoubtedly a better place with you in it ;).

### My Method and Disclaimer:
I am relatively new to ROS, and would consider myself a functional programmer, rather than a distinguised one. I have taken the learning-by-doing approach during this project. I therefore can't guarantee the efficiency or efficacy of any of my code. If you have any suggestions for improvements, then please drop them in the issues section of this repo or request a merge if you've implemented it yourself. If you have any other questions or ideas, then please feel free to email me at gregory.baker@brl.ac.uk.

### Other Useful Resources:
Understanding PID better: 
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- When I first setup the PID control for velocity control of my wheels, the tuning was terrible (started with kp = 1, ki = 1, kd = 0.01) - I hadn't tuned a PID controller before and didn't know where to begin with hand-tuning. But after maybe 30 mins - 1 hour of just playing around with the numbers I arrived at one that worked pretty well. It was actually pure integral control with a gain of Ki = 100. That was a good day.

Communication between Pi and Teensy: 
http://wiki.ros.org/rosserial

Understanding ros_control better: 
https://fjp.at/posts/ros/ros-control/ and https://fjp.at/projects/diffbot/ros-packages/control/
- I'm much more familiar with C# and Python than I am with C++. It was therfore a bit of a shock when I realised that the interface between the hardware (which I'd just about struggled through with the help of the santized Arduino IDE) and the diff_drive_controller (that I knew I probably needed to use) was all in C++. ros_control is an immensely useful tool and I know it has saved me a lot of pain in the long run, but boy-oh-boy was it a nightmare to get my head around. The above links are a good starting point. The 'Diffbot' project actually has a lot of overlap with mine as well. Once I'd eventually got my head around some amount of ros_control I decided that the best course of action was to restructure my microcontroller code so I could copy the [jackal implemntation](https://github.com/jackal/jackal_robot/tree/melodic-devel/jackal_base) as closely as possible. This involved defining ros message types that look remarkably similar to [jackal_msgs](https://github.com/jackal/jackal/tree/melodic-devel/jackal_msgs) and loading the message defs to the Arduino IDE using rosserial_arduino (see [here](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)).

Robot Localization
