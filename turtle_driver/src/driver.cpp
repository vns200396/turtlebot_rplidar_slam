#include "turtle_driver/driver.h"
 



BaseController::BaseController(ros::NodeHandle& nh,const char *port, int baud, float timeout, std::string _base_frame )
                                :nh_(nh),Controller(port,baud,&wheelLeft, &wheelRight)
{
    PID_RATE = 100;
    PID_INTERVAL = 1000/100;

    base_frame = _base_frame;
    ros::param::get("~base_controller_rate", rate);
    ros::param::get("~base_controller_timeout", timeout);
    stopped = false;

    ros::param::get("~wheel_diameter", wheel_diameter);
    ros::param::get("~wheel_track", wheel_track);
    ros::param::get("~encoder_resolution", encoder_resolution);
    ros::param::get("~gear_reduction", gear_reduction);
    ros::param::get("~Kp", Kp);
    ros::param::get("~Ki", Ki);
    ros::param::get("~Kd", Kd);
    ros::param::get("~Ko", Ko);

    ros::param::get("~encoder_min", encoder_min);
    ros::param::get("~encoder_max", encoder_max);

    ros::param::get("~useSonar", useSonar);
    ros::param::get("~useImu", useImu);
    ros::param::get("~usedebug", usedebug);

    nh_.param<std::string>("/turtle_driver/tf_parent_frame_id", tf_parent_frame_id, "imu_base");
    nh_.param<std::string>("/turtle_driver/tf_frame_id", tf_frame_id, "imu");
    nh_.param<std::string>("/turtle_driver/imu_frame_id", imu_frame_id, "imu_base");
    nh_.param<double>("/turtle_driver/time_offset_in_seconds", time_offset_in_seconds, 0.0);

    //Set up PID parameters and check for missing values
    PIDEnable(Kp, Ki, Kd);

    //How many encoder ticks are there per meter?
    ticks_per_meter = encoder_resolution * gear_reduction  / (wheel_diameter * PI);

    //What is the maximum acceleration we will tolerate when changing wheel speeds?
    max_accel = (accel_limit * ticks_per_meter) / rate;

    //Track how often we get a bad encoder count (if any)
    bad_encoder_count = 0;

    ros::param::get("~accel_limit", accel_limit);
    ros::param::get("~motors_reversed", motors_reversed);

    encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
    encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;
    ros::param::get("~encoder_low_wrap", encoder_low_wrap);
    ros::param::get("~encoder_high_wrap", encoder_high_wrap);
    l_wheel_mult=0;
    r_wheel_mult=0;

    now = ros::Time::now();
    t_then =now;
    t_delta = ros::Duration(1.0 / rate);

    ultrasonic_ranger = 500;

    safe_ranger_0=3;
    safe_ranger_1=5;
    safe_ranger_2=8;

    //Internal data 
    enc_left = 0;          //encoder readings
    enc_right = 0;
    x = 0;                     // position in xy plane
    y = 0;
    th = 0;                   // rotation in radians
    v_left = 0;
    v_right = 0;
    v_des_left = 0;             // cmd_vel setpoint
    v_des_right = 0;
    vel_driver = 0;
    radius = 0;
    drive(0, 0);
    last_cmd_vel = now;

    bool useSmoother = false;
    ros::param::get("~useSmoother", useSmoother);
    //Subscriptions
    if(useSmoother == true)
    {
        cmd_vel_sub = nh_.subscribe("smoother_cmd_vel", 10, &BaseController::cmdVelCallback, this);
    }
    else
    {
        cmd_vel_sub = nh_.subscribe("cmd_vel", 10, &BaseController::cmdVelCallback, this);
    }

    //Clear any old odometry info
    reset_encoders();

    //Set up the odometry broadcaster
    odomPub = nh_.advertise<nav_msgs::Odometry>("odom", 5);

    ROS_INFO("Started base controller for a base of %f m wide with %f ticks per revolution %f  ticks per meter", wheel_track, encoder_resolution,ticks_per_meter);
    ROS_INFO("Publishing odometry data at: %d  Hz using  %s as base frame", rate, base_frame.c_str());

    ros::Publisher lEncoderPub = nh_.advertise<std_msgs::Int16>("lEncoderPub", 1);
    ros::Publisher rEncoderPub = nh_.advertise<std_msgs::Int16>("rEncoderPub", 1);
    ros::Publisher lPidoutPub = nh_.advertise<std_msgs::Int16>("lPidoutPub", 1);
    ros::Publisher rPidoutPub = nh_.advertise<std_msgs::Int16>("rPidoutPub", 1);
    lVelPub = nh_.advertise<std_msgs::Int16>("lVelPub", 1);
    rVelPub = nh_.advertise<std_msgs::Int16>("rVelPub", 1);

    imu_pub = nh_.advertise<sensor_msgs::Imu>("imu", 50);
    imu_angle_pub = nh_.advertise<std_msgs::Float32>("imu_angle", 50);
    service = nh_.advertiseService("set_zero_orientation", &BaseController::set_zero_orientation, this);
    imu_current_time = ros::Time::now();
    imu_nxt_time = ros::Time::now();
    imu_delt_time =  ros::Duration(0.01);

    ROS_INFO_STREAM("tf_parent_frame_id: " << tf_parent_frame_id.c_str());
    ROS_INFO_STREAM("tf_frame_id: " << tf_frame_id.c_str());
    ROS_INFO_STREAM("imu_frame_id: " << imu_frame_id.c_str());

    if(useSonar)
    {
        sonarPub =  nh_.advertise<std_msgs::Int16>("sonarPub", 1);
    }

}

void BaseController::cmdVelCallback(const geometry_msgs::Twist& twist_aux)
{
    geometry_msgs::Twist req = twist_aux;
    last_cmd_vel = ros::Time::now();
    double right,left;
    double vel_x,vel_th;

    vel_x = req.linear.x;        // m/s
    vel_th = req.angular.z;      // rad/s

    if(useSonar)
    {
        if ((ultrasonic_ranger <= safe_ranger_0) && (x>0))
        {
            x=0;
        }
        else if ((ultrasonic_ranger <= safe_ranger_1) && (x>0.05))
        {
            x=0.05;
        }
        else if ((ultrasonic_ranger <= safe_ranger_2) && (x>0.15))
        {
            x=0.08;
        }
    }

    if (vel_x == 0)
    {
        //Turn in place
        right = vel_th * wheel_track  * gear_reduction / 2.0;
        left = -right;
    }
    else if(vel_th == 0)
    {
        //Pure forward/backward motion
        left = right = vel_x;
    }
    else
    {
        //Rotation about a point in space
        left = vel_x - vel_th * wheel_track  * gear_reduction / 2.0;
        right = vel_x + vel_th * wheel_track  * gear_reduction / 2.0;
    }


    v_des_left = int(left * ticks_per_meter / PID_RATE);
    v_des_right = int(right * ticks_per_meter / PID_RATE);

    vel_driver = vel_x;
    radius= vel_th;
    //debugger();
}

void BaseController::poll(void)
{
    uint8_t result=0;

    requestController(t_delta);
    result = controllerRespond();

    if(result == CMD_REQUEST_SONAR_ENC)
    {
        double dt;
        double dright,dleft; //distance righ, distance left
        uint16_t left_enc = encoder_sonar.EncoderLeft;
        uint16_t right_enc  = encoder_sonar.EncoderRight;
        ultrasonic_ranger = encoder_sonar.Distance;

        now = ros::Time::now();
        dt = now.toSec() - t_then.toSec();
        t_then = ros::Time::now();

        
        //Calculate odometry
        if (enc_left == 0)
        {
            dright = 0;
            dleft = 0;
        }
        else
        {
            if (left_enc < encoder_low_wrap && enc_left > encoder_high_wrap)
                { l_wheel_mult = l_wheel_mult + 1;  }    
            else if (left_enc > encoder_high_wrap && enc_left < encoder_low_wrap) 
                { l_wheel_mult = l_wheel_mult - 1;  }
            else
                {  l_wheel_mult = 0; }

            if (right_enc < encoder_low_wrap && enc_right > encoder_high_wrap)
                {   r_wheel_mult = r_wheel_mult + 1;    }
            else if (right_enc > encoder_high_wrap && enc_right < encoder_low_wrap)
                {   r_wheel_mult = r_wheel_mult - 1;    }
            else
                {   r_wheel_mult = 0;  }    
            //dright = (right_enc - enc_right) / ticks_per_meter
            //dleft = (left_enc - enc_left) / ticks_per_meter
            dleft = 1.0 * (left_enc + l_wheel_mult * (encoder_max - encoder_min)-enc_left) / ticks_per_meter;
            dright = 1.0 * (right_enc + r_wheel_mult * (encoder_max - encoder_min)-enc_right) / ticks_per_meter;
        }


        enc_right = right_enc;
        enc_left = left_enc;
        
        double dxy_ave = (dright + dleft) / 2.0;
        double dth = (dleft - dright) / wheel_track;
        double vxy = dxy_ave / dt;
        double vth = dth / dt;
            
        if (dxy_ave != 0)
        {
            double dx = -cos(dth) * dxy_ave;
            double dy = sin(dth) * dxy_ave;
            x += (cos(th) * dx - sin(th) * dy);
            y += (sin(th) * dx + cos(th) * dy);
        }


        if (dth != 0)
        {
            th += dth;
        }

        debugger();
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(th);
        
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = sin(th / 2.0);
        quaternion.w = cos(th / 2.0);

        // Create the odometry transform frame broadcaster.
        if(useImu == false)
        {
            geometry_msgs::TransformStamped odom_transform;
            odom_transform.header.frame_id = "odom";
            odom_transform.child_frame_id = base_frame;
            odom_transform.transform.translation.x = x;
            odom_transform.transform.translation.y = y;
            odom_transform.transform.translation.z = 0.0;
            odom_transform.transform.rotation = quaternion;
            odom_transform.header.stamp = ros::Time::now();
            odomBroadcaster.sendTransform(odom_transform);
        }

        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.child_frame_id = base_frame;
        odom.header.stamp = now;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = quaternion;
        
        if (driverinfor2.SpeedRPMLeft == 0 && driverinfor2.SpeedRPMRight == 0){
            odom.pose.covariance[0] = 1e-9;
            odom.pose.covariance[7] = 1e-3;
            odom.pose.covariance[8] = 1e-9;
            odom.pose.covariance[14] = 1e6;
            odom.pose.covariance[21] = 1e6;
            odom.pose.covariance[28] = 1e6;
            odom.pose.covariance[35] = 1e-9;
            odom.twist.covariance[0] = 1e-9;
            odom.twist.covariance[7] = 1e-3;
            odom.twist.covariance[8] = 1e-9;
            odom.twist.covariance[14] = 1e6;
            odom.twist.covariance[21] = 1e6;
            odom.twist.covariance[28] = 1e6;
            odom.twist.covariance[35] = 1e-9;
        }
        else{
            odom.pose.covariance[0] = 1e-3;
            odom.pose.covariance[7] = 1e-3;
            odom.pose.covariance[8] = 0.0;
            odom.pose.covariance[14] = 1e6;
            odom.pose.covariance[21] = 1e6;
            odom.pose.covariance[28] = 1e6;
            odom.pose.covariance[35] = 1e3;
            odom.twist.covariance[0] = 1e-3;
            odom.twist.covariance[7] = 1e-3;
            odom.twist.covariance[8] = 0.0;
            odom.twist.covariance[14] = 1e6;
            odom.twist.covariance[21] = 1e6;
            odom.twist.covariance[28] = 1e6;
            odom.twist.covariance[35] = 1e3;
        }
        
        odom.twist.twist.linear.x = vxy;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = vth;

        odomPub.publish(odom);


        std_msgs::Int16 msg_v_left;
        std_msgs::Int16 msg_v_right;
        msg_v_left.data = driverinfor2.SpeedRPMLeft;
        msg_v_right.data = driverinfor2.SpeedRPMRight;
        lVelPub.publish(msg_v_left);
        rVelPub.publish(msg_v_right);    

        //Set motor speeds in mm/s
        if (stopped != true)
        {
            drive(vel_driver, radius);
        }
        else
        {
             drive(0, 0);
        }
    }

    if(result == CMD_REQUEST_ACC_IMU && useImu == true)
    {
        sync_imu();
    }


    if(result == CMD_REQUEST_SONAR_ENC && useSonar == true)
    {
        std_msgs::Int16 sonar_dist;
        sonar_dist.data = encoder_sonar.Distance;
        sonarPub.publish(sonar_dist);
    }
}

void BaseController::drive(double vel, double radius)
{
     int velocity = (int)(fabs(vel)*1000); //m to mm
     int radiusMM = (int)(1000*fabs(radius)* wheel_track/2); //rad to mm

    if (vel < 0 && radius == 0)
    {
        setCarBackoff(velocity);
    }
    else if (vel > 0 && radius < 0) // turn right
    {
        setCarUpperRight(velocity,radiusMM);
    }
    else if (vel > 0 && radius > 0) //turn left
    {

        setCarUpperLeft(velocity,radiusMM);
    }
    else if (vel < 0 && radius > 0) //turn n right
    {
        setCarLowerRight(velocity,radiusMM);
    }
    else if (vel < 0 && radius < 0) //turn n left
    {
        setCarLowerLeft(velocity,radiusMM);
    }
    else if (vel == 0 && radius > 0) // rotate left in place
    {
        setCarRotateLeft(radiusMM);
    }
    else if (vel == 0 && radius < 0) // rotate right in place
    {
        setCarRotateRight(radiusMM);
    }
    else //(vel > 0 && radius == 0)
    {
        setCarAdvance(velocity);
    }
}

bool BaseController::set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

void BaseController::sync_imu(void)
{
    double acc_x,acc_y,acc_z;
    double roll,pitch,yaw;
    double resutl=0;

    acc_x = get_acceleration_X();
    acc_y = get_acceleration_Y();
    acc_z = get_acceleration_Z();

    resutl = acc_x+acc_y+acc_z;
    if(!imu_driver.Status_IMU || resutl ==0)
    {
        requestCommand(CMD_RESET_IMU);
        ROS_ERROR_STREAM("IMU Fail!");
        if(!flag_resetIMU)
        {
            flag_resetIMU =1;
            requestCommand(CMD_RESET_IMU);
        }
        return;
    }

    roll = getRoll();
    pitch = getPitch();
    yaw = getYaw();

    // Low-pass filter
    rollF = 0.94 * rollF + 0.06 * roll;
    pitchF = 0.94 * pitchF + 0.06 * pitch;
    yawF = 0.94 * yawF + 0.06 * yaw;

    // imu_current_time = ros::Time::now();
    // if(imu_current_time < imu_nxt_time) return;


    tf::Quaternion orientation = tf::createQuaternionFromYaw(yawF);
    std_msgs::Float32 imu_angle_mgs;
    imu_angle_mgs.data = get_acceleration_Z();
    imu_angle_pub.publish(imu_angle_mgs);

    if (!zero_orientation_set)
    {
        zero_orientation = orientation;
        zero_orientation_set = true;
    }

    tf::Quaternion differential_rotation;
    differential_rotation = zero_orientation.inverse() * orientation;
    // calculate measurement time
    ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);
    // publish imu message
    sensor_msgs::Imu imu_msgs;
    imu_msgs.header.stamp = measurement_time;
    imu_msgs.header.frame_id = imu_frame_id;
    quaternionTFToMsg(differential_rotation, imu_msgs.orientation);

    // i do not know the orientation covariance
    imu_msgs.orientation_covariance[0] = 1000000;
    imu_msgs.orientation_covariance[1] = 0;
    imu_msgs.orientation_covariance[2] = 0;
    imu_msgs.orientation_covariance[3] = 0;
    imu_msgs.orientation_covariance[4] = 1000000;
    imu_msgs.orientation_covariance[5] = 0;
    imu_msgs.orientation_covariance[6] = 0;
    imu_msgs.orientation_covariance[7] = 0;
    imu_msgs.orientation_covariance[8] = 0.000001;
    // angular velocity is not provided
    //imu.angular_velocity_covariance[0] = -1;
    
    imu_msgs.angular_velocity.x = rollF;
    imu_msgs.angular_velocity.y = pitchF;
    imu_msgs.angular_velocity.z = yawF;

    imu_msgs.linear_acceleration.x = acc_x;
    imu_msgs.linear_acceleration.y = acc_y;
    imu_msgs.linear_acceleration.z = acc_z;
    // ROS_INFO("linear_acceleration.x: %f", imu_msgs.linear_acceleration.x );
    // ROS_INFO("linear_acceleration.y: %f", imu_msgs.linear_acceleration.y );
    // ROS_INFO("linear_acceleration.z: %f", imu_msgs.linear_acceleration.z );

    // Orientation (quarternion)
    // imu_msgs.orientation.x = 0;
    // imu_msgs.orientation.y = 0;
    // imu_msgs.orientation.z = 0;
    // imu_msgs.orientation.w = 0;
    //imu_msgs.linear_acceleration_covariance[0] = -1;
    imu_pub.publish(imu_msgs);
    // publish tf transform
    tf::Transform transform;
    transform.setRotation(differential_rotation);
    imuBroadcaster.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
      
    //imu_nxt_time = imu_current_time + imu_delt_time;
}
void BaseController::debugger(void)
{
	static double time_begin = ros::Time::now().toSec();
	double time_now = ros::Time::now().toSec();

    if(usedebug == false) return;

	if(time_now - time_begin > 1)
	{
		time_begin = ros::Time::now().toSec();

	 	//ROS_INFO("Spawning turtle [turtle1] at x=[%f], y=[%f], theta=[%f]", x,y,th);
        //ROS_INFO("v_des_left = %d \r\n v_des_right = %d", v_des_left,v_des_right);
       // ROS_INFO("dxy_ave = %f", dxy_ave);
        // controller_debugger();
        // ROS_INFO("\r\n left_enc = %d \r\n right_enc = %d \r\n enc_left = %d \r\n enc_right = %d \r\n l_wheel_mult = %f \r\n r_wheel_mult = %f", encoder_sonar.EncoderLeft,encoder_sonar.EncoderRight, enc_left, enc_right, l_wheel_mult, r_wheel_mult);
        // ROS_INFO("\r\n dleft = %f \r\n dright = %f \r\n dxy_ave = %f", dleft,dright, dxy_ave);
        ROS_INFO("\r\n x = %f \r\n y = %f \r\n th = %f", x,y, th);
        
        // ROS_INFO("angular_velocity.x: %f", rollF);
        // ROS_INFO("angular_velocity.y: %f", pitchF );
        // ROS_INFO("angular_velocity.z: %f", yawF );
        // ROS_INFO("linear_acceleration.x: %f", get_acceleration_X());
        // ROS_INFO("linear_acceleration.y: %f", get_acceleration_Y());
        // ROS_INFO("linear_acceleration.z: %f", get_acceleration_Z());
	}
}



uint8_t flag_reconfig=0;
double kp,ki,kd;
void dyminic_reconfig_callback(turtle_driver::pid_controlConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f %f %f %f %d",
    config.KP_param,
    config.KI_param,
    config.KD_param,
    config.Sample_time,
    config.Speed);

    kp =  config.KP_param;
    ki =  config.KI_param;
    kd =  config.KD_param;
    flag_reconfig =1;
} 

int main (int argc, char** argv){
    ros::init(argc, argv, "turtle_driver");
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    //Declares the message to be sent
    geometry_msgs::Twist cmd_vel;

    std::string port_serial = "/dev/ttyACM0";
    int baudrate = 115200;
    std::string base_frame = "base_link";
    int rate=0;
    int sensorstate_rate=0;
    bool use_base_controller = false;

    ros::param::get("~port", port_serial);
    ros::param::get("~baud", baudrate);

    ros::param::get("~base_frame", base_frame);
    //Overall loop rate: should be faster than fastest sensor rate
    ros::param::get("~rate", rate);
    //Rate at which summary SensorState message is published. Individual sensors publish
    //at their own rates. 
    ros::param::get("~sensorstate_rate", sensorstate_rate);

    ros::param::get("~use_base_controller", use_base_controller);

    //A cmd_vel publisher so we can stop the robot when shutting down
    cmd_vel_pub.publish(cmd_vel);
    BaseController bc(n,port_serial.c_str(),baudrate,0.1,base_frame);

    dynamic_reconfigure::Server<turtle_driver::pid_controlConfig>server; 
    dynamic_reconfigure::Server<turtle_driver::pid_controlConfig>::CallbackType f;
    f = boost::bind(dyminic_reconfig_callback, _1, _2);
    server.setCallback(f);
    flag_reconfig=0;
    while(ros::ok())
    {
        ros::spinOnce();
        bc.poll();
        if(flag_reconfig)
        {
            flag_reconfig=0;
            bc.PIDEnable(kp,ki,kd);
        }
    }
}

