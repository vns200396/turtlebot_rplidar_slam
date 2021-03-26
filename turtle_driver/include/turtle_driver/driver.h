#ifndef  DRIVER_H
#define  DRIVER_H

#include <ros/ros.h>


#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Quaternion.h>
#include <stdio.h>
#include <cmath>

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <turtle_driver/controller.h>
#include <turtle_driver/pid_controlConfig.h>




class BaseController: public Controller
{
public:
    BaseController(ros::NodeHandle& nh,const char *port, int baud, float timeout, std::string _base_frame );
    //~BaseController();
    void poll(void);
    void stop(void);
    void cmdVelCallback(const geometry_msgs::Twist& twist_aux);
private:
    MotorWheel wheelLeft;
    MotorWheel wheelRight;
    std::string base_frame;
    uint16_t PID_RATE;
    uint16_t PID_INTERVAL;
    int rate=10;
    float timeout=1.0;
    bool stopped = false;
    bool useSonar =false;
    bool useImu =false;
    double wheel_diameter;
    double wheel_track;
    double encoder_resolution;
    double gear_reduction;
    double Kp;
    double Ki;
    double Kd;
    double Ko;

    double accel_limit=0.1;
    bool motors_reversed=false;

    double ticks_per_meter;
    double max_accel;
    int encoder_min = 0;
    int encoder_max = 65535;
    double bad_encoder_count=0;
    double encoder_low_wrap;
    double encoder_high_wrap;
    double l_wheel_mult;
    double r_wheel_mult;

    ros::Time now;
    ros::Time t_next;
    ros::Duration t_delta;
    ros::Time t_then;

    uint16_t ultrasonic_ranger;
    uint8_t safe_ranger_0;
    uint8_t safe_ranger_1;
    uint8_t safe_ranger_2;

    uint16_t enc_left;
    uint16_t enc_right;
    double x;
    double y;
    double th;
    int16_t  v_left;
    int16_t  v_right;
    int16_t  v_des_left;
    int16_t  v_des_right;
    double vel_driver;
    double radius;
    ros::Time last_cmd_vel;
    bool usedebug=false;

    double distanceLeft;
    double distanceRight;

    std::string tf_parent_frame_id;
    std::string tf_frame_id;
    std::string imu_frame_id;
	bool zero_orientation_set = false;
    double time_offset_in_seconds =0;
    double rollF=0.0;
    double pitchF=0.0;
    double yawF=0.0;
    ros::Time imu_current_time; 
    ros::Time imu_nxt_time;
    ros::Duration imu_delt_time;

    void drive(double vel, double radius);
    void sync_imu(void);
    bool set_zero_orientation(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    void debugger();

protected:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher sonarPub;
    ros::Publisher lVelPub;
    ros::Publisher rVelPub;
    ros::Publisher odomPub;
    tf::TransformBroadcaster odomBroadcaster; 
    tf::TransformBroadcaster imuBroadcaster;
    tf::Quaternion orientation;
    tf::Quaternion zero_orientation;
    ros::Publisher imu_pub;
    ros::Publisher imu_angle_pub;
    ros::ServiceServer service;
};



#endif
