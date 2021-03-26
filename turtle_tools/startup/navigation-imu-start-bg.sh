#!/bin/bash
ProgName="navigation_demo_imu_web.launch"
num=$(ps aux|grep ${ProgName} |grep -v grep|wc -l)

if [ $num -gt 0 ] 
then 
   echo "${ProgName} has been launched!"
else
   /home/ubuntu/catkin_ws/src/turtle_tools/startup/navigation-imu-start-fg.sh>/dev/null 2>&1 &
   echo "${ProgName} is starting"
fi
