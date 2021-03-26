#include "serialt.h"


void sayhello()
{
    ROS_INFO_STREAM("Hello Son");
}


void tim1(void)
{
    static 	double begin = ros::Time::now().toSec();
    double now = ros::Time::now().toSec();
    now = now - begin;
    if(now >= 0.5)
    {
        ROS_INFO_STREAM("Time 1: " << now );
        begin = ros::Time::now().toSec();
    }
}

void tim2(void)
{
    static 	double begin = ros::Time::now().toSec();
    double now = ros::Time::now().toSec();
    now = now - begin;
    if(now >= 2)
    {
        ROS_INFO_STREAM("Time 2: " << now );
        begin = ros::Time::now().toSec();
    }
}
