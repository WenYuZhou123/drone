#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <simple_follower/position.h> // 导入自定义的追踪位置消息类型
#include <nav_msgs/Odometry.h>//里程计信息格式

#define max_vel 0.6
#define min_vel 0.2
using namespace std;

float  delta_x, delta_y,delta_z;
 
float init_position_x,init_position_y,init_position_z;
int flag_init_position = 0;

//当前的坐标位置信息
float map_local_x,map_local_y,map_local_z;

mavros_msgs::PositionTarget setpoint_raw;//速度、高度、偏航角控制
 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


nav_msgs::Odometry local_pos;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	local_pos = *msg;
    map_local_x       = local_pos.pose.pose.position.x;
    map_local_y       = local_pos.pose.pose.position.y;
    map_local_z       = local_pos.pose.pose.position.z;
    if (flag_init_position==0 && (local_pos.pose.pose.position.z!=0))
    {
	   init_position_x = local_pos.pose.pose.position.x;
	   init_position_y = local_pos.pose.pose.position.y;
	   init_position_z = local_pos.pose.pose.position.z;
       flag_init_position = 1;		    
     }
}

simple_follower::position landmark;
void landmark_cb(const simple_follower::position::ConstPtr& msg)
{
    landmark = *msg;
}
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh;
 	
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber landmark_sub = nh.subscribe<simple_follower::position>("landmark_position", 10, landmark_cb);
 
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);   

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Rate rate(20.0);  
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //期望的飞行位置，相对于上电时刻，其中x表示正东方向，y表示正北方向
    delta_x = 0; 
    delta_y = 0;
    delta_z = 3; 
    
	setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
    setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = init_position_x + delta_x;
	setpoint_raw.position.y = init_position_y + delta_y;
	setpoint_raw.position.z = init_position_z + delta_z;
	mavros_setpoint_pos_pub.publish(setpoint_raw);
	
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
     
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
 
    //起飞    
    while(ros::ok())
    {
        setpoint_raw.position.x = init_position_x + delta_x;
		setpoint_raw.position.y = init_position_y + delta_y;
		setpoint_raw.position.z = init_position_z + delta_z;
		printf("init_position_z = %f",init_position_z);
		printf("   setpoint_raw.position.z = %f\n",setpoint_raw.position.z);
	
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
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		             //	    break;
		        }
		        	last_request = ros::Time::now();
			}
		}
	    //
	     if(ros::Time::now() - last_request > ros::Duration(10.0))
             break;
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();

	//控制降落部分
	while(ros::ok())      
	{	
		//如果找到地标，控制方向
		if(landmark.iffind)
		{
            printf("Object_detected\r\n");
			//无人机左右移动速度控制
			if(landmark.angleX > 5)
            {
				if(0.05*(landmark.angleX - 5)<=min_vel)
				{
					setpoint_raw.velocity.y = min_vel;	
				}	
			    else if((0.05*(landmark.angleX - 5)>=max_vel))
				{
					setpoint_raw.velocity.y = max_vel;	
				}
				else
				{
					setpoint_raw.velocity.y = 0.05*(landmark.angleX - 5);
				}							
			}			
	    	else if(landmark.angleX < -5)
			{
				if(0.05*(landmark.angleX + 5)>= -min_vel)
				{
					setpoint_raw.velocity.y = -min_vel;	
				}	
			    else if((0.05*(landmark.angleX + 5)<=-max_vel))
				{
					setpoint_raw.velocity.y = -max_vel;	
				}
				else
				{
					setpoint_raw.velocity.y = 0.05*(landmark.angleX + 5);
				}	
			}
		    else
			{
				setpoint_raw.velocity.y = 0;
			}

	        //无人机前后移动速度控制
			if(landmark.angleY > 5)
			{
				if(0.05*(landmark.angleY - 5)<=min_vel)
				{
					setpoint_raw.velocity.x = min_vel;	
				}	
			    else if((0.05*(landmark.angleY - 5)>=max_vel))
				{
					setpoint_raw.velocity.x = max_vel;	
				}
				else
				{
					setpoint_raw.velocity.x = 0.05*(landmark.angleY - 5);
				}
			}
			else if(landmark.angleY < -5)
			{
				if(0.05*(landmark.angleY + 5)>= -min_vel)
				{
					setpoint_raw.velocity.x = -min_vel;	
				}	
			    else if((0.05*(landmark.angleY + 5)<=-max_vel))
				{
					setpoint_raw.velocity.x = -max_vel;	
				}
				else
				{
				    setpoint_raw.velocity.x = 0.05*(landmark.angleY + 5);
				}			
			}
			else
			{
				setpoint_raw.velocity.x = 0;
			}
								
			setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
			//setpoint_raw.velocity.x = 0;
			//setpoint_raw.velocity.y = 0;
			setpoint_raw.position.z = init_position_z+delta_z;
			mavros_setpoint_pos_pub.publish(setpoint_raw);
						
		    //如果位置很正开始降落
			cout<<landmark.angleX<<endl;
			cout<<landmark.angleY<<endl;
			
			if((landmark.angleX<=5 && landmark.angleX>=-5) && (landmark.angleY<=5 && landmark.angleY>=-5))
	        {	
	        	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			    setpoint_raw.coordinate_frame = 8;
				setpoint_raw.velocity.x = 0;
				setpoint_raw.velocity.y = 0;
				setpoint_raw.position.z = init_position_z+delta_z;
			    mavros_setpoint_pos_pub.publish(setpoint_raw);
				ROS_INFO("arrive goals");			
			//	break;	
			}
			mavros_setpoint_pos_pub.publish(setpoint_raw);
    	}
    	//如果找不到红色色块,目前设置为保持不动，具体可以看实际效果测试
    	else
		{
		   	setpoint_raw.type_mask = 1 + 2 + /*4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
			setpoint_raw.velocity.x = 0;
			setpoint_raw.velocity.y = 0;
			setpoint_raw.position.z = init_position_z + delta_z;
			mavros_setpoint_pos_pub.publish(setpoint_raw);	     
	    }

    	mavros_setpoint_pos_pub.publish(setpoint_raw);
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("AUTO.LAND");

	offb_set_mode.request.custom_mode = "AUTO.LAND";
	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	{
		ROS_INFO("Autoland enabled");
		last_request = ros::Time::now();
	}
  
    return 0;
}

