FROM osrf/ros:melodic-desktop-full

SHELL [ "/bin/bash", "-c" ]
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt update && apt upgrade -y
RUN apt install python3-pip -y
RUN pip3 install numpy


RUN mkdir -p  /root/catkin_ws/src/
COPY . /root/catkin_ws/src

RUN source /opt/ros/melodic/setup.bash && cd /root/catkin_ws && catkin_make && source devel/setup.bash

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
