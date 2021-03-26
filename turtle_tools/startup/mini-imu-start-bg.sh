#!/bin/bash
num=$(ps aux|grep demo_imu.launch |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   echo "demo_imu.launch has been launched!"
else
   /home/ubuntu/catkin_ws/src/turtle_tools/startup/mini-imu-start-fg.sh>/dev/null 2>&1 &
   echo "demo_imu.launch is starting"
fi
