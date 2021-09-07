FROM ros:noetic-ros-base

ENV ROS_WS=/home/catkin_ws

RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

RUN apt update && apt install -y \
    python3-pip

RUN pip3 install sparkfun_pi_servo_hat

COPY vr_pan_tilt_autophat $ROS_WS/src/vr_pan_tilt_autophat 

RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make

RUN echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc 

SHELL ["/bin/bash", "-c"]

RUN source ~/.bashrc
