#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <string>

#define MAX_ERROR 50
#define VEL_SET   0.15
#define ALTITUDE  0.40

using namespace std;

yolov8_ros_msgs::BoundingBoxes object_pos;
nav_msgs::Odometry local_pos;
mavros_msgs::State current_state;  
mavros_msgs::PositionTarget setpoint_raw;
 
//检测到的物体坐标值
double position_detec_x = 0;
double position_detec_y = 0;
std::string Class = "no_object";

std::string target_object_id = "eight";

void state_cb(const mavros_msgs::State::ConstPtr& msg);

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);

void object_pos_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg);

int main(int argc, char **argv)
{
	//防止中文输出乱码
	setlocale(LC_ALL, "");

    //初始化节点，名称为visual_throw
    ros::init(argc, argv, "follow_yolov8");
    
    //创建句柄
    ros::NodeHandle nh;
	 
	//订阅无人机状态话题
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
		
	//订阅无人机实时位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, local_pos_cb);
    
     //订阅实时位置信息
    ros::Subscriber object_pos_sub = nh.subscribe<yolov8_ros_msgs::BoundingBoxes>("object_position", 100, object_pos_cb);
		
	//发布无人机位置控制话题
	ros::Publisher  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
		
	//发布无人机多维控制话题
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);   
		               
	//请求无人机解锁服务        
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	//请求无人机设置飞行模式，本代码请求进入offboard
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//请求控制舵机客户端
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    //循环频率
    ros::Rate rate(20.0); 
    
    

    ros::param::get("target_object_id", target_object_id);
   
    //等待连接到PX4无人机
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
	setpoint_raw.coordinate_frame = 8;
	setpoint_raw.position.x = 0;
	setpoint_raw.position.y = 0;
	setpoint_raw.position.z = 0 + ALTITUDE;
	mavros_setpoint_pos_pub.publish(setpoint_raw);
 
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    //请求offboard模式变量
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    //请求解锁变量
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //请求进入offboard模式并且解锁无人机，15秒后退出，防止重复请求       
   
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
	    if(ros::Time::now() - last_request > ros::Duration(5.0))
	    	break;
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    } 
	
    while(ros::ok())
    {      
    
   		//此处表示识别到launch文件中指定的目标
		//if(object_pos.bounding_boxes[0].Class == "chair")
		if(Class == target_object_id)
        {
        	ROS_INFO("识别到目标，采用速度控制进行跟随");
			//摄像头向下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向
			//无人机左右移动速度控制
			if(position_detec_x-320 >= MAX_ERROR)
			{
				setpoint_raw.velocity.y =  -VEL_SET;
			}					
			else if(position_detec_x-320 <= -MAX_ERROR)
			{
				setpoint_raw.velocity.y =  VEL_SET;
			}	
			else
			{
				setpoint_raw.velocity.y =  0;
			}
			//无人机前后移动速度控制
			if(position_detec_y-240 >= MAX_ERROR)
			{
				setpoint_raw.velocity.x =  -VEL_SET;
			}					
			else if(position_detec_y-240 <= -MAX_ERROR)
			{
				setpoint_raw.velocity.x =  VEL_SET;
			}	
			else
			{
				setpoint_raw.velocity.x =  0;
			}
				
		}
		else
		{
			setpoint_raw.velocity.x =  0;
			setpoint_raw.velocity.y =  0;
		}
		setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 8;
		setpoint_raw.velocity.x = 0;
		setpoint_raw.position.z = 0 + ALTITUDE;
		setpoint_raw.yaw        = 0;
	    mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
}

void object_pos_cb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	object_pos = *msg;
    position_detec_x = object_pos.bounding_boxes[0].xmin;
    position_detec_y = object_pos.bounding_boxes[0].ymin;
    Class =            object_pos.bounding_boxes[0].Class;
}


