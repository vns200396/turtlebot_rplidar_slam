/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "turtle_driver/controller.h"
#include <dynamic_reconfigure/server.h>
#include <turtle_driver/pid_controlConfig.h>

MotorWheel wheelLeft;
MotorWheel wheelRight;

Controller driver("/dev/ttyUSB0",115200 ,&wheelLeft,&wheelRight);




void goAhead(unsigned int speedMMPS)
{
    driver.setCarAdvance(0);
    driver.setCarSpeedMMPS(speedMMPS,300);
}

void turnLeft(unsigned int speedMMPS)
{

}


void turnRight(unsigned int speedMMPS)
{


}


void rotateRight(unsigned int speedMMPS)
{

}


void rotateLeft(unsigned int speedMMPS)
{

}

int speed=500;
void callback(turtle_driver::pid_controlConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f %f %d",
    config.KP_param,
    config.KI_param,
    config.KD_param,
    config.Sample_time,
    config.Speed);

    speed = config.Speed;

    // if
    driver.PIDEnable(config.KP_param,config.KI_param,config.KD_param,config.Sample_time);

    driver.requestCommand(CMD_SET_PID);
} 

int main (int argc, char** argv){
    ros::init(argc, argv, "turtle_driver");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<turtle_driver::pid_controlConfig>server; 
    dynamic_reconfigure::Server<turtle_driver::pid_controlConfig>::CallbackType f;
    f = boost::bind(callback, _1, _2);
    server.setCallback(f);

    driver.PIDEnable(0,0,0,0.01);
    while(ros::ok())
    {
        ros::spinOnce();
        
        goAhead(speed);
        driver.spin();
        //ROS_INFO_STREAM("Testing serial port");
        //comm.waitDriverResponse(&packet, 20);
        //loop_rate.sleep();
    }
}






/*
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.readline(1,"");
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

*/