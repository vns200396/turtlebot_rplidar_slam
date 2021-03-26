#!/bin/bash

log_file=/tmp/turtle.log
DATE=`date`

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

interface=eth0
echo "$DATE: turtle-start on interface $interface" >> $log_file

export ROS_IP=`ifconfig $interface | grep -o 'inet addr:[^ ]*' | cut -d: -f2`

echo "$DATE: turtle-start setting ROS_IP=$ROS_IP" >> $log_file

if [ "$ROS_IP" = "" ]; then
    echo "$DATE: turtle-start can't run with empty ROS_IP." >> $log_file
    exit 1
fi

roslaunch turtle_driver demo.launch
