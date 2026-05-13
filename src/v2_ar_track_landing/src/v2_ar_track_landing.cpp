#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#define MAX_ERROR 0.20
#define VEL_SET   0.15

using namespace std;

mavros_msgs::PositionTarget setpoint_raw;


bool marker_found = false;
int current_target_id = 8;
float last_position_x = 0;
float last_position_y = 0;

//这里高度采用float类型，因为高度要变化控制，因此不是用宏定义
float altitude = 1.0;

//定义变量，用于接收无人机的里程计信息
tf::Quaternion quat; 
double roll, pitch, yaw;
float init_position_x_take_off =0;
float init_position_y_take_off =0;
float init_position_z_take_off =0;
bool  flag_init_position = false;
nav_msgs::Odometry local_pos;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

mavros_msgs::State current_state;  
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//检测到的物体坐标值
double position_detec_x = 0;
double position_detec_y = 0;
double position_detec_z = 0;
ar_track_alvar_msgs::AlvarMarker marker;	
void ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	int count = msg->markers.size();
	if(count!=0)
	{
		for(int i = 0; i<count; i++)
		{
			marker = msg->markers[i];			
			if(marker.id == current_target_id)
			{
				marker_found = true;
			}
		}
		//此处根本摄像头安装方向，进行静态坐标转换
        position_detec_x = marker.pose.pose.position.x;
        position_detec_y = marker.pose.pose.position.y;
        position_detec_z = marker.pose.pose.position.z; 
	}
	else
	{
		marker_found = false;
	}
}

int main(int argc, char **argv)
{
    //初始化节点，名称为v2_ar_track_landing
    ros::init(argc, argv, "v2_ar_track_landing");
    
    //创建句柄
    ros::NodeHandle nh;
	 
	//订阅无人机状态话题
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
		
	//订阅无人机实时位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 100, local_pos_cb);
    
    //订阅实时位置信息
    ros::Subscriber ar_pos_sub = nh.subscribe("/ar_pose_marker", 100, ar_marker_cb);
			
	//发布无人机多维控制话题
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);   
		               
	//请求无人机解锁服务        
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	//请求无人机设置飞行模式，本代码请求进入offboard
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //循环频率
    ros::Rate rate(20.0); 
   
    //等待连接到PX4无人机
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x =init_position_x_take_off + 0;
	setpoint_raw.position.y =init_position_y_take_off + 0;
	setpoint_raw.position.z =init_position_z_take_off + altitude;
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
    
    ros::Time last_request         = ros::Time::now();

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
           	flag_init_position = false;		    
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
		        flag_init_position = false;		    
			}
		}
	    //1、添加高度判断，使得无人机跳出模式切换循环
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -altitude)<0.5)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				break;
			}
		}
		//2、添加时间判断，使得无人机跳出模式切换循环
		if(ros::Time::now() - last_request > ros::Duration(8.0))
		{
			break;
		}		
		//此处添加是为增加无人机的安全性能，在实际测试过程中，采用某款国产的GPS和飞控，气压计和GPS定位误差极大，
		//导致了无人机起飞后直接飘走，高度和位置都不正常，无法跳出模式循环，导致遥控且无法接管
		//因此增加了时间判断，确保无人机在切入offboard模式和解锁后，确保任何情况下，8秒后遥控器都能切入其他模式接管无人机	
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的				
		setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.x =init_position_x_take_off + 0;
		setpoint_raw.position.y =init_position_y_take_off + 0;
		last_position_x = local_pos.pose.pose.position.x;
		last_position_y = local_pos.pose.pose.position.y;	
		setpoint_raw.position.z =init_position_z_take_off + altitude;		
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }   	
    last_request = ros::Time::now();
    bool flag_arrive = false;
    while(ros::ok())
    {      
		if(marker_found && flag_arrive==false)
        {
			if(fabs(position_detec_x) < MAX_ERROR && fabs(position_detec_y) < MAX_ERROR)
			{	
				//记录水平位置，防止风或者GPS误差
				last_position_x = local_pos.pose.pose.position.x;
				last_position_y = local_pos.pose.pose.position.y;				
				//此处添加高度判断，如果高度小于1M则进入降落模式，否则进行动态调整，每次降低0.5米高度进行一次调整
				if(altitude<0.6)
				{
					flag_arrive = true;
				}
				//递减降低高度降落
				else
				{
				 	altitude = altitude - 0.01;
				}
				setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
				setpoint_raw.coordinate_frame = 1;
				setpoint_raw.position.x = last_position_x;
				setpoint_raw.position.y = last_position_y;
				setpoint_raw.position.z = init_position_z_take_off + altitude;
			}
			else
			{
				//摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
				//无人机左右移动速度控制
				if(position_detec_x >= MAX_ERROR)
				{
					setpoint_raw.velocity.y =  -VEL_SET;
				}					
				else if(position_detec_x <= -MAX_ERROR)
				{
					setpoint_raw.velocity.y =  VEL_SET;
				}	
				else
				{
					setpoint_raw.velocity.y =  0;
				}								  
				//无人机前后移动速度控制
				if(position_detec_y >= MAX_ERROR)
				{
					setpoint_raw.velocity.x =  -VEL_SET;
				}
				else if(position_detec_y <= -MAX_ERROR)
				{
					setpoint_raw.velocity.x =  VEL_SET;
				}
				else
				{
					setpoint_raw.velocity.x =  0;
				}
			    setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
				setpoint_raw.coordinate_frame = 1;
				setpoint_raw.position.z = init_position_z_take_off + altitude;
			}			
			//更新水平位置，确保丢失目标情况下能够保持位置控制
			last_position_x = local_pos.pose.pose.position.x;
			last_position_y = local_pos.pose.pose.position.y;					
		}
		//没识别到二维码，则保持当前位置不动
		else
		{
			setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
			setpoint_raw.coordinate_frame = 1;
			setpoint_raw.velocity.x =  0;
			setpoint_raw.velocity.y =  0;
			setpoint_raw.position.x =last_position_x;
			setpoint_raw.position.y =last_position_y;
			setpoint_raw.position.z =init_position_z_take_off + altitude;		
		}
		//此处使用高度低于起飞初始值，可以利用位置环的PID控制，有效的抵消风的影响
		if(flag_arrive == true)
		{
			setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 1;
			setpoint_raw.position.x =last_position_x;
			setpoint_raw.position.y =last_position_y;
			setpoint_raw.position.z =init_position_z_take_off - 1;		
		}			
		mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}






